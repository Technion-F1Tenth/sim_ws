#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <Eigen/Core>
#include <fstream>
#include <sstream>
#include <limits>
#include <cmath>

using Vec2   = Eigen::Vector2d;
using Points = std::vector<Vec2, Eigen::aligned_allocator<Vec2>>; // alignment-safe

enum LoadMode{
    CENTERLINE,
    RACELINE,
};

struct LoadRequest{
    std::string filepath;
    LoadMode mode;
};

struct Path {
  Points pts;
  std::vector<double> s;
  bool closed{true};  // racelines wrap around; paths from incoming_path_topic are open
};

class PurePursuitNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    double lookahead_distance_{1.5};
    double wheelbase_{0.28};
    double speed_cmd_{1.0};
    double max_steer_{0.4189}; // ~24 deg
    Path path;
    bool is_active_{true};

    // New: stop-at-end behavior
    bool stop_at_end_{true};          // whether to stop when reaching the final path point
    double stop_distance_{0.20};     // threshold (meters) to consider the robot at the goal
    bool stopped_at_goal_{false};    // internal flag to avoid re-sending stop command
    size_t progress_idx_{0};         // open paths: closest index never moves backwards


    static double yaw_from_quat(const geometry_msgs::msg::Quaternion &q)
    {
        // ZYX yaw from quaternion
        return std::atan2(2.0*(q.w*q.z + q.x*q.y),
                          1.0 - 2.0*(q.y*q.y + q.z*q.z));
    }

    static double wrap_angle(double a)
    {
        while (a >  M_PI) a -= 2*M_PI;
        while (a < -M_PI) a += 2*M_PI;
        return a;
    }
    // Service callbacks to pause/resume the controller
     std::shared_ptr<std_srvs::srv::Trigger::Response> pause_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        response->success = true;
        response->message = "PurePursuitNode paused, speed set to 0.";
        RCLCPP_INFO(this->get_logger(), "PurePursuitNode paused via service call.");
        is_active_ = false;
        ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
        ackermann_msg.drive.speed = 0.0;
        ackermann_msg.drive.steering_angle = 0.0;
        drive_pub_->publish(ackermann_msg);
        return response;
    }
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!msg) {
            RCLCPP_WARN(this->get_logger(), "Received null Path message");
            return;
        }
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty Path message");
            return;
        }

        Path new_path;
        new_path.pts.reserve(msg->poses.size());
        for (const auto &ps : msg->poses) {
            const auto &p = ps.pose.position;
            new_path.pts.emplace_back(p.x, p.y);
        }

        // compute cumulative arc-length s
        new_path.s.assign(new_path.pts.size(), 0.0);
        for (size_t i = 1; i < new_path.pts.size(); ++i) {
            new_path.s[i] = new_path.s[i - 1] + (new_path.pts[i] - new_path.pts[i - 1]).norm();
        }

        // Planned paths go from the robot to a goal: no wrap-around at the end
        new_path.closed = false;

        // replace active path
        path = std::move(new_path);
        // Reset stop flag and (re)enable controller when a new path arrives
        stopped_at_goal_ = false;
        progress_idx_ = 0;
        is_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Loaded path from topic with %zu points", path.pts.size());
    }



    std::shared_ptr<std_srvs::srv::Trigger::Response> resume_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        response->success = true;
        response->message = "PurePursuitNode resumed.";
        RCLCPP_INFO(this->get_logger(), "PurePursuitNode resumed via service call.");
        is_active_ = true;
        return response;
    }

    // March forward along the polyline from 'start_idx' until cumulative arc-length >= L.
    // Wraps around on closed tracks; stops at the last point on open paths.
    size_t forward_lookahead_index(const Path& p, size_t start_idx, double L) const
    {
        const size_t N = p.pts.size();
        if (N < 2) return start_idx;

        size_t i = start_idx;
        double acc = 0.0;
        // Cap iterations to N to avoid infinite loops on small/degenerate paths
        for (size_t steps = 0; steps < N && acc < L; ++steps) {
            if (!p.closed && i == N - 1) break;
            size_t j = (i + 1) % N;
            acc += (p.pts[j] - p.pts[i]).norm();
            i = j;
        }
        return i; // index of the lookahead point along the path
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!is_active_) {
            // Controller is paused

            return;
        }
        const std::size_t N = path.pts.size();
        if (N < 2) return;

        const auto &pos = msg->pose.pose.position;
        Vec2 current_pos(pos.x, pos.y);

        // 1) Find closest point along the path (by Euclidean distance). On open paths
        //    search only ahead of the progress so far, so a path that passes near
        //    itself can't pull the robot back to an earlier part.
        const size_t search_from = path.closed ? 0 : std::min(progress_idx_, N - 1);
        double min_dist2 = std::numeric_limits<double>::max();
        size_t closest_idx = search_from;
        for (size_t i = search_from; i < N; ++i) {
            const double d2 = (path.pts[i] - current_pos).squaredNorm();
            if (d2 < min_dist2) {
                min_dist2 = d2;
                closest_idx = i;
            }
        }
        if (!path.closed) progress_idx_ = closest_idx;

        const double min_dist = std::sqrt(min_dist2);
        const Vec2   closest_point = path.pts[closest_idx];

        // If configured, stop at the final point: within stop_distance, or (open
        // paths) once the robot has driven past the end of the last segment.
        if (stop_at_end_ && !stopped_at_goal_) {
            const Vec2 last_point = path.pts.back();
            const double dist_to_last = (last_point - current_pos).norm();
            const bool passed_end = !path.closed && closest_idx == N - 1 &&
                (current_pos - last_point).dot(last_point - path.pts[N - 2]) >= 0.0;
            if (dist_to_last <= stop_distance_ || passed_end) {
                // publish zero-speed command and mark stopped
                ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
                ackermann_msg.drive.speed = 0.0;
                ackermann_msg.drive.steering_angle = 0.0;
                drive_pub_->publish(ackermann_msg);
                stopped_at_goal_ = true;
                // pause controller to avoid further movement until resumed or a new path arrives
                is_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Reached final path point (dist=%.3fm). Stopping robot.", dist_to_last);
                return;
            }
        }

        // 2) Choose lookahead point by marching forward along the polyline. Near the
        //    end of an open path, extend the last segment so the target stays a full
        //    lookahead ahead instead of collapsing onto (and swinging around) the goal.
        const size_t look_idx = forward_lookahead_index(path, closest_idx, lookahead_distance_);
        Vec2 lookahead_point = path.pts[look_idx];
        if (!path.closed && look_idx == N - 1) {
            const double remaining = (path.s[N - 1] - path.s[closest_idx]) + min_dist;
            const Vec2 seg = path.pts[N - 1] - path.pts[N - 2];
            if (remaining < lookahead_distance_ && seg.norm() > 1e-9) {
                lookahead_point += seg.normalized() * (lookahead_distance_ - remaining);
            }
        }

        // 3) Vehicle yaw and transform target to vehicle frame
        const double yaw = yaw_from_quat(msg->pose.pose.orientation);

        const Vec2 d = lookahead_point - current_pos; // world-frame vector to target
        const double c = std::cos(-yaw), s = std::sin(-yaw);
        const Vec2 pL(c*d.x() - s*d.y(), s*d.x() + c*d.y()); // into vehicle frame

        // 4) Pure Pursuit curvature and steering
        double delta = 0.0;
        const double Ld2 = pL.squaredNorm();
        if (Ld2 > 1e-9) {
            // Standard PP: kappa = 2*y_L / Ld^2 ; delta = atan(L * kappa)
            const double kappa = 2.0 * pL.y() / Ld2;
            delta = std::atan(wheelbase_ * kappa);
        }
        // Clamp steering to vehicle limits
        if (delta >  max_steer_) delta =  max_steer_;
        if (delta < -max_steer_) delta = -max_steer_;

        // 5) Publish command
        ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
        ackermann_msg.drive.speed = speed_cmd_;
        ackermann_msg.drive.steering_angle = delta;
        drive_pub_->publish(ackermann_msg);

        // 6) Helpful logs, throttled to 1 Hz (odom arrives at ~200 Hz in sim)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "pos(%.2f, %.2f) -> closest[%zu](%.2f, %.2f) d=%.2f | look[%zu](%.2f, %.2f) | yaw=%.2f rad | steer=%.3f",
                    current_pos.x(), current_pos.y(),
                    closest_idx, closest_point.x(), closest_point.y(), min_dist,
                    look_idx, lookahead_point.x(), lookahead_point.y(),
                    yaw, delta);
    }

    // Centerline CSVs are comma-separated, raceline CSVs ';'-separated: decide
    // from the first non-comment line.
    static LoadMode detect_load_mode(const std::string &filepath)
    {
        std::ifstream file(filepath);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            return line.find(',') != std::string::npos ? LoadMode::CENTERLINE : LoadMode::RACELINE;
        }
        return LoadMode::RACELINE;
    }

    Path load_path(LoadRequest req){
        Path path;
        switch (req.mode)
        {
        case LoadMode::CENTERLINE:
        {
            // f1tenth_racetracks centerline: "# x_m, y_m, w_tr_right_m, w_tr_left_m"
            RCLCPP_INFO(this->get_logger(), "Centerline mode selected, loading...");
            std::ifstream file(req.filepath);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open centerline file: %s", req.filepath.c_str());
                return path;
            }
            std::string line;
            while (std::getline(file, line)) {
                if (line.empty() || line[0] == '#') continue;
                std::stringstream ss(line);
                std::string col;
                std::vector<std::string> cols;
                while (std::getline(ss, col, ',')) {
                    cols.push_back(col);
                }
                if (cols.size() < 2) continue;
                try {
                    path.pts.emplace_back(std::stod(cols[0]), std::stod(cols[1]));
                } catch (const std::exception &) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
                    continue;
                }
            }
            path.s.assign(path.pts.size(), 0.0);
            for (size_t i = 1; i < path.pts.size(); ++i) {
                path.s[i] = path.s[i-1] + (path.pts[i] - path.pts[i-1]).norm();
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu centerline points", path.pts.size());
            if (!path.pts.empty()) {
                RCLCPP_INFO(this->get_logger(), "First point: x=%.3f, y=%.3f", path.pts[0].x(), path.pts[0].y());
            }
        }
        break;

        case LoadMode::RACELINE:
        {
            RCLCPP_INFO(this->get_logger(), "Raceline mode selected, loading...");
            std::ifstream file(req.filepath);
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open raceline file: %s", req.filepath.c_str());
                return path;
            }
            std::string line;
            // Skip header if present
            std::getline(file, line);
            while (std::getline(file, line)) {
                if (line.empty()) continue;
                std::stringstream ss(line);
                std::string col;
                std::vector<std::string> cols;
                while (std::getline(ss, col, ';')) {
                    cols.push_back(col);
                }
                if (cols.size() < 4) continue;
                try {
                    double x = std::stod(cols[2]);
                    double y = std::stod(cols[3]);
                    path.pts.emplace_back(x, y);

                    if (cols.size() >= 3) {
                        path.s.push_back(std::stod(cols[1]));
                    }
                } catch (const std::exception &) {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse line: %s", line.c_str());
                    continue;
                }
            }
            // If 's' not given, compute cumulative arc-length
            if ((path.s.size() != path.pts.size()) && !path.pts.empty()) {
                path.s.clear();
                path.s.resize(path.pts.size(), 0.0);
                for (size_t i = 1; i < path.pts.size(); ++i) {
                    path.s[i] = path.s[i-1] + (path.pts[i] - path.pts[i-1]).norm();
                }
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu raceline points", path.pts.size());
            if (!path.pts.empty()) {
                RCLCPP_INFO(this->get_logger(), "First point: x=%.3f, y=%.3f", path.pts[0].x(), path.pts[0].y());
            }
        }
        break;

        default:
            RCLCPP_WARN(this->get_logger(), "No loading mode selected, path is empty");
            break;
        }
        return path;
    }



public:
    PurePursuitNode()
    : Node("pure_pursuit_node")
    {
        // Topics
        this->declare_parameter("odom_topic", "ego_racecar/odom");
        const std::string odom_topic = this->get_parameter("odom_topic").as_string();
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&PurePursuitNode::odom_callback, this, std::placeholders::_1));

        // Default matches the cmd-mux contract: controllers publish to /safe_drive,
        // and safety_node is the sole writer of /drive. Bringup launches override
        // this explicitly, but a bare `ros2 run` should be safe by default too.
        this->declare_parameter("drive_topic", "/safe_drive");
        const std::string drive_topic = this->get_parameter("drive_topic").as_string();
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic,10);


        this->declare_parameter("pause_topic", "/pause");
        const std::string pause_topic = this->get_parameter("pause_topic").as_string();
        pause_service_ = this->create_service<std_srvs::srv::Trigger>(
            pause_topic, std::bind(&PurePursuitNode::pause_callback, this, std::placeholders::_1, std::placeholders::_2));

        this->declare_parameter("resume_topic", "/resume");
        const std::string resume_topic = this->get_parameter("resume_topic").as_string();
        resume_service_ = this->create_service<std_srvs::srv::Trigger>(
            resume_topic, std::bind(&PurePursuitNode::resume_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Vehicle parameters
        this->declare_parameter("wheelbase", 0.28);
        wheelbase_ = this->get_parameter("wheelbase").as_double();

        // Control parameters
        this->declare_parameter("lookahead_distance", 0.8); // << was uninitialized before
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();

        this->declare_parameter("speed", 1.0);
        speed_cmd_ = this->get_parameter("speed").as_double();

        this->declare_parameter("max_steer", 45 * M_PI / 180.0);
        max_steer_ = this->get_parameter("max_steer").as_double();

        // Path parameters & loading
        this->declare_parameter<std::string>("raceline_path", "");
        auto raceline_path = this->get_parameter("raceline_path").as_string();
        this->declare_parameter<bool>("init_path", false);
        const bool init_path = this->get_parameter("init_path").as_bool();
        this->declare_parameter<std::string>("incoming_path_topic", "/planned_path");
        const std::string incoming_path_topic = this->get_parameter("incoming_path_topic").as_string();

        // Stop-at-end parameters
        this->declare_parameter<bool>("stop_at_end", true);
        stop_at_end_ = this->get_parameter("stop_at_end").as_bool();
        this->declare_parameter<double>("stop_distance", 0.20);
        stop_distance_ = this->get_parameter("stop_distance").as_double();

         // Transient-local: a one-shot/latched path publisher (e.g. a plan
         // loaded from a file) may publish before this node subscribes.
         const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
         path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
             incoming_path_topic, path_qos,
             std::bind(&PurePursuitNode::path_callback, this, std::placeholders::_1));

        if (init_path)
        {
            LoadRequest req;
            req.filepath = raceline_path;
            req.mode = detect_load_mode(raceline_path);
            path = load_path(req);
            if (path.pts.size() < 2) {
                RCLCPP_WARN(this->get_logger(), "Path has < 2 points; controller will idle until a valid path is loaded.");
            }
        }

        RCLCPP_INFO(this->get_logger(),
            "PurePursuit started. lookahead=%.2f m, speed=%.2f m/s, max_steer=%.3f rad, wheelbase=%.3f m",
            lookahead_distance_, speed_cmd_, max_steer_, wheelbase_);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
