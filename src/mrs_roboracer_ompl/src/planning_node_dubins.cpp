#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <Eigen/Core>
#include <fstream>
#include <sstream>
#include <limits>
#include <cmath>
#include <mutex>
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PlanningNode : public rclcpp::Node{
  private:
    // Private members would go here
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;
    rclcpp::TimerBase::SharedPtr map_request_timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // Store read-only shared_ptrs to the latest messages
    nav_msgs::msg::Odometry::ConstSharedPtr current_odom_;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_pose_;

    // Vehicle parametrs
    double min_turning_radius_ = 0.7; // meters

    // Planner parameters
    double goal_tolerance_ = 0.05;
    double planning_timeout_sec_ = 10.0;
    // Store processed map data
    double resolution_;
    unsigned long width_;
    unsigned long height_;
    Eigen::Vector2d origin_;
    Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> occupancy_grid_;


    std::mutex data_mutex_;

    void plan_path_(const geometry_msgs::msg::PoseStamped &goal) {
      // Placeholder for path planning logic
      geometry_msgs::msg::PoseWithCovariance start = current_odom_->pose;
      RCLCPP_INFO(this->get_logger(), "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
                  start.pose.position.x, start.pose.position.y,
                  goal.pose.position.x, goal.pose.position.y);

      ob::StateSpacePtr space = std::make_shared<ob::DubinsStateSpace>(min_turning_radius_, /*isSymmetric=*/true);
      ompl::base::RealVectorBounds bounds =ob::RealVectorBounds(2);
      double x0B, y0B, x1B, y1B;
      x0B = origin_[0];
      y0B = origin_[1];
      x1B = origin_[0] + width_ * resolution_;
      y1B = origin_[1] + height_ * resolution_;
      bounds.setLow(0, x0B); bounds.setLow(1, y0B);
      bounds.setHigh(0, x1B); bounds.setHigh(1, y1B);
      space->as<ob::DubinsStateSpace>()->setBounds(bounds);

      // 3) SimpleSetup
      og::SimpleSetup ss(space);

      // 4) State validity checker using the occupancy grid
      auto svc = [this](const ob::State *state) -> bool {
        const auto *se2 = state->as<ob::SE2StateSpace::StateType>();
        const double x = se2->getX();
        const double y = se2->getY();
        int mx, my;
        Eigen::Vector2<double> pos = Eigen::Vector2d(x, y);
        return is_free_space(pos);
      };
      ss.setStateValidityChecker(svc);

      // 5. Make sure the OMPL discretization and the occupancy grid are consistent (BIG DEAL!)
      auto si = ss.getSpaceInformation();

      const double extent = space->getMaximumExtent();

      // target: longest segment ≈ 0.5 * resolution, expressed as a fraction of extent
      double frac = (0.5 * resolution_) / std::max(extent, 1e-6);

      // clamp to [1e-4, 0.05] to avoid extremes
      frac = std::clamp(frac, 1e-4, 0.05);

      // apply to both the space’s longest-valid-segment fraction and the
      // motion validator’s state validity checking resolution
      space->setLongestValidSegmentFraction(frac);
      si->setStateValidityCheckingResolution(frac);

      // 6) Start & goal (x,y,yaw)
      double start_yaw = yaw_from_quaternion_(start.pose.orientation);
      double goal_yaw = yaw_from_quaternion_(goal.pose.orientation);
      ob::ScopedState<ob::SE2StateSpace> s(space), g(space);
      s->setX(start.pose.position.x);  s->setY(start.pose.position.y);  s->setYaw(start_yaw);
      g->setX(goal.pose.position.x);  g->setY(goal.pose.position.y);  g->setYaw(goal_yaw);

      ss.setStartAndGoalStates(s, g, goal_tolerance_);

      // 7) Planner + objective
      ss.setPlanner(std::make_shared<og::RRTstar>(si));
      ss.getProblemDefinition()->setOptimizationObjective(
          std::make_shared<ob::PathLengthOptimizationObjective>(si));

      // 8) Solve
      ob::PlannerStatus solved = ss.solve(planning_timeout_sec_);
      if (!solved) {
        RCLCPP_WARN(this->get_logger(), "No solution found in %.2fs.", planning_timeout_sec_);
        return;
      }

      // 9) Retrieve and optionally simplify/interpolate the path
      og::PathGeometric path = ss.getSolutionPath();


      // 10) Convert to nav_msgs/Path
      nav_msgs::msg::Path ros_path;
      ros_path.header.frame_id = "/map";
      ros_path.header.stamp = this->now();

      for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        const auto *se2 = path.getState(i)->as<ob::SE2StateSpace::StateType>();
        geometry_msgs::msg::PoseStamped p;
        p.header = ros_path.header;
        p.pose.position.x = se2->getX();
        p.pose.position.y = se2->getY();
        p.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, se2->getYaw());
        p.pose.orientation = tf2::toMsg(q);

        ros_path.poses.push_back(std::move(p));
      }

      path_pub_->publish(ros_path);
      RCLCPP_INFO(this->get_logger(), "Published path with %zu poses.", ros_path.poses.size());
    }

    double yaw_from_quaternion_(const geometry_msgs::msg::Quaternion &q) {
      // Convert quaternion to yaw angle
      double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      return std::atan2(siny_cosp, cosy_cosp);
    }

    geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw) {
      // Construct a quaternion from a yaw (rotation around Z)
      geometry_msgs::msg::Quaternion q;
      q.x = 0.0;
      q.y = 0.0;
      q.z = std::sin(yaw * 0.5);
      q.w = std::cos(yaw * 0.5);
      return q;
    }


    void odom_callback_(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_odom_ = msg;
    }


    void save_map(const nav_msgs::msg::MapMetaData &info, const std::vector<int8_t> &map_data) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      resolution_ = info.resolution;
      width_ = info.width;
      height_ = info.height;


      const std::size_t expected_size = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
      if (map_data.size() != expected_size) {
        RCLCPP_WARN(this->get_logger(), "save_map: incoming map data size (%zu) does not match width*height (%zu)", map_data.size(), expected_size);
      }

      int rows = static_cast<int>(height_);
      int cols = static_cast<int>(width_);

      occupancy_grid_.resize(rows, cols);

      if (!map_data.empty()) {
        Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mapped(map_data.data(), rows, cols);
        occupancy_grid_ = mapped;
      } else {
        occupancy_grid_.setZero();
      }

      origin_ = Eigen::Vector2d(info.origin.position.x, info.origin.position.y);
      RCLCPP_INFO(this->get_logger(), "Map updated... Ready to plan");
    }

    void map_callback_(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      save_map(msg->info, msg->data);
    }

    void goal_pose_callback_(const geometry_msgs::msg::PoseStamped &msg) {
      goal_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(msg);
      RCLCPP_INFO(this->get_logger(), "Received new goal pose. Starting to plan");
      plan_path_(msg);
    }

    void clicked_point_callback_(const geometry_msgs::msg::PointStamped &msg) {
      double x = msg.point.x;
      double y = msg.point.y;
      Eigen::Vector2d point_position = Eigen::Vector2d(x, y);
      // Verify if point is occupied in occupancy grid
      if (is_free_space(point_position)) {
        RCLCPP_INFO(this->get_logger(), "Clicked point (%.2f, %.2f) is in free space.", x, y);
      } else {
        RCLCPP_INFO(this->get_logger(), "Clicked point (%.2f, %.2f) is occupied.", x, y);
      }
    }

    bool is_free_space(Eigen::Vector2d point) {
      std::lock_guard<std::mutex> lock(data_mutex_);

      if (resolution_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "is_free_space: resolution is invalid (%.5f)", resolution_);
        return false;
      }
      if (occupancy_grid_.size() == 0 || width_ == 0 || height_ == 0) {
        RCLCPP_WARN(this->get_logger(), "is_free_space: occupancy grid not initialized yet");
        return false;
      }

      double dx = point[0] - origin_[0];
      double dy = point[1] - origin_[1];

      int col = static_cast<int>(std::floor(dx / resolution_));
      int row = static_cast<int>(std::floor(dy / resolution_));

      if (row < 0 || row >= occupancy_grid_.rows() || col < 0 || col >= occupancy_grid_.cols()) {
        RCLCPP_DEBUG(this->get_logger(), "is_free_space: point (%.3f, %.3f) -> idx (r=%d,c=%d) out of bounds",
                      point[0], point[1], row, col);
        return false; // treat out-of-map as not free
      }

      int8_t cell = occupancy_grid_(row, col);

      // Interpret occupancy: 0 = free, 100 = occupied, -1 = unknown
      if (cell == 0) return true;
      // treat unknown and anything > 0 as occupied for safety
      return false;
    }

  // --- One-shot request once the service is ready ---
  void try_request_map_() {
      if (!map_client_->wait_for_service(std::chrono::seconds(0))) {
        // Not up yet; keep waiting
        return;
      }
      map_request_timer_->cancel();

      auto req = std::make_shared<nav_msgs::srv::GetMap::Request>(); // Request has no fields
      map_client_->async_send_request(
        req,
        std::bind(&PlanningNode::on_map_response_, this, std::placeholders::_1)
      );
      RCLCPP_INFO(get_logger(), "Requested static map from /map_server/map");
    }

  void on_map_response_(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
      auto resp = future.get();
      RCLCPP_INFO(get_logger(), "Saving. Got map %u x %u @ %.3fm",
                  resp->map.info.width, resp->map.info.height, resp->map.info.resolution);
      save_map(resp->map.info, resp->map.data);
      RCLCPP_INFO(get_logger(), "Got map %u x %u @ %.3fm",
                  resp->map.info.width, resp->map.info.height, resp->map.info.resolution);
    }

  public:
    PlanningNode()
    : Node("planning_node"), resolution_(0.0), width_(0), height_(0), origin_(0.0, 0.0)
    {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/ego_racecar/odom", 10, std::bind(&PlanningNode::odom_callback_, this, std::placeholders::_1));
      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/map", 10, std::bind(&PlanningNode::map_callback_, this, std::placeholders::_1));

      goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/goal_pose", 10, std::bind(&PlanningNode::goal_pose_callback_, this, std::placeholders::_1));

      clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, std::bind(&PlanningNode::clicked_point_callback_, this, std::placeholders::_1));

      map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

      // Try until service is up, then request once
      map_request_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PlanningNode::try_request_map_, this)
      );


      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 1);


    }
};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world mrs_roboracer_ompl package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
