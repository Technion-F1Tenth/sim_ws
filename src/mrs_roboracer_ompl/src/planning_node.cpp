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
#include <array>
#include <algorithm>
#include <Eigen/Core>
#include <fstream>
#include <sstream>
#include <limits>
#include <cmath>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#ifdef HAS_RCS
#include "RCSStar.h"
#endif

#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;
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

    // Planner parameters
    std::string planner_type_ = "dubins";
    double min_turning_radius_ = 1.0;
    double path_resolution_ = 0.1;
    double goal_tolerance_ = 0.05;
    double planning_timeout_sec_ = 60.0;

    // RCS hyperparameters
    int rcs_min_resolution_ = 0;
    int rcs_discrete_level_ = 2;
    int rcs_lookahead_ = 0;
    int rcs_node_cost_factor_ = 1;
    int rcs_coarse_control_factor_ = 0;
    double rcs_ics_reject_radius_ = 0.0;
    double rcs_speed_cutoff_ = 1e-10;
    double rcs_epsilon_ = 0.5;
    double rcs_goal_tolerance_ = 1.5;

    // RCS sweep parameters
    bool rcs_sweep_enabled_ = false;
    std::string rcs_sweep_config_path_;

    // Vehicle dynamics parameters
    double wheelbase_ = 0.2032;
    double max_velocity_ = 4.0;
    double max_steering_angle_ = 0.52;
    double max_acceleration_ = 0.5;
    double max_steering_rate_ = 0.035;
    double propagation_step_size_ = 0.3;
    int control_duration_steps_ = 16;
    // Store processed map data
    double resolution_;
    unsigned long width_;
    unsigned long height_;
    Eigen::Vector2d origin_;
    Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> occupancy_grid_;
    Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> inflated_grid_;
    double robot_radius_ = 0.3;
    int8_t occupancy_threshold_ = 50;
    bool unknown_is_free_ = false;

    std::mutex data_mutex_;

    bool ready_to_plan_() {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!current_odom_) {
        RCLCPP_WARN(this->get_logger(), "No odometry received yet; cannot plan.");
        return false;
      }
      if (inflated_grid_.size() == 0) {
        RCLCPP_WARN(this->get_logger(), "No map received yet; cannot plan.");
        return false;
      }
      return true;
    }

    void publish_path_(const std::vector<std::array<double, 3>> &poses) {
      nav_msgs::msg::Path ros_path;
      ros_path.header.frame_id = "map";
      ros_path.header.stamp = this->now();
      for (const auto &xyyaw : poses) {
        geometry_msgs::msg::PoseStamped p;
        p.header = ros_path.header;
        p.pose.position.x = xyyaw[0];
        p.pose.position.y = xyyaw[1];
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, xyyaw[2]);
        p.pose.orientation = tf2::toMsg(q);
        ros_path.poses.push_back(std::move(p));
      }
      path_pub_->publish(ros_path);
      RCLCPP_INFO(this->get_logger(), "Published path with %zu poses.", ros_path.poses.size());
    }

    // Geometric RRT* over Dubins curves: forward-only, curvature-limited paths
    // that pure pursuit can follow. Path length is minimized for the whole budget.
    void plan_dubins_path_(const geometry_msgs::msg::PoseStamped &goal) {
      geometry_msgs::msg::PoseWithCovariance start = current_odom_->pose;
      RCLCPP_INFO(this->get_logger(), "Planning Dubins path (r_min %.2f m) from (%.2f, %.2f) to (%.2f, %.2f)",
                  min_turning_radius_, start.pose.position.x, start.pose.position.y,
                  goal.pose.position.x, goal.pose.position.y);

      auto space = std::make_shared<ob::DubinsStateSpace>(min_turning_radius_);
      ob::RealVectorBounds bounds(2);
      bounds.setLow(0, origin_[0]);
      bounds.setLow(1, origin_[1]);
      bounds.setHigh(0, origin_[0] + width_ * resolution_);
      bounds.setHigh(1, origin_[1] + height_ * resolution_);
      space->setBounds(bounds);

      og::SimpleSetup ss(space);
      ss.setStateValidityChecker([this](const ob::State *state) -> bool {
        const auto *se2 = state->as<ob::SE2StateSpace::StateType>();
        return is_free_space(Eigen::Vector2d(se2->getX(), se2->getY()));
      });
      // Check motions at half a map cell so edges can't jump over thin walls
      auto si = ss.getSpaceInformation();
      si->setStateValidityCheckingResolution(
          std::clamp(0.5 * resolution_ / std::max(space->getMaximumExtent(), 1e-6), 1e-4, 0.05));

      ob::ScopedState<ob::SE2StateSpace> s(space), g(space);
      s->setXY(start.pose.position.x, start.pose.position.y);
      s->setYaw(yaw_from_quaternion_(start.pose.orientation));
      g->setXY(goal.pose.position.x, goal.pose.position.y);
      g->setYaw(yaw_from_quaternion_(goal.pose.orientation));
      if (!si->isValid(s.get())) {
        RCLCPP_WARN(this->get_logger(), "Start pose is inside an (inflated) obstacle; not planning.");
        return;
      }
      if (!si->isValid(g.get())) {
        RCLCPP_WARN(this->get_logger(), "Goal pose is inside an (inflated) obstacle; not planning.");
        return;
      }
      ss.setStartAndGoalStates(s, g, goal_tolerance_);
      ss.getProblemDefinition()->setOptimizationObjective(
          std::make_shared<ob::PathLengthOptimizationObjective>(si));
      ss.setPlanner(std::make_shared<og::RRTstar>(si));

      ob::PlannerStatus solved = ss.solve(planning_timeout_sec_);
      if (solved != ob::PlannerStatus::EXACT_SOLUTION) {
        RCLCPP_WARN(this->get_logger(), "No exact solution found in %.2fs (%s).",
                    planning_timeout_sec_, solved.asString().c_str());
        return;
      }

      ss.simplifySolution();
      og::PathGeometric &path = ss.getSolutionPath();
      const double length = path.length();
      path.interpolate(std::max<unsigned int>(2, static_cast<unsigned int>(std::ceil(length / path_resolution_))));

      std::vector<std::array<double, 3>> poses;
      for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        const auto *se2 = path.getState(i)->as<ob::SE2StateSpace::StateType>();
        poses.push_back({se2->getX(), se2->getY(), se2->getYaw()});
      }
      RCLCPP_INFO(this->get_logger(), "Dubins path: %.2f m", length);
      publish_path_(poses);
    }

    void plan_path_(const geometry_msgs::msg::PoseStamped &goal) {
      geometry_msgs::msg::PoseWithCovariance start = current_odom_->pose;
      RCLCPP_INFO(this->get_logger(), "Planning path from (%.2f, %.2f) to (%.2f, %.2f)",
                  start.pose.position.x, start.pose.position.y,
                  goal.pose.position.x, goal.pose.position.y);

      // 1) Build compound state space: SE2 (x, y, theta) + RealVector(2) (v, phi)
      auto se2_space = std::make_shared<ob::SE2StateSpace>();
      ob::RealVectorBounds se2_bounds(2);
      se2_bounds.setLow(0, origin_[0]);
      se2_bounds.setLow(1, origin_[1]);
      se2_bounds.setHigh(0, origin_[0] + width_ * resolution_);
      se2_bounds.setHigh(1, origin_[1] + height_ * resolution_);
      se2_space->setBounds(se2_bounds);

      auto vel_steer_space = std::make_shared<ob::RealVectorStateSpace>(2);
      ob::RealVectorBounds vs_bounds(2);
      vs_bounds.setLow(0, 0.0);
      vs_bounds.setHigh(0, max_velocity_);
      vs_bounds.setLow(1, -max_steering_angle_);
      vs_bounds.setHigh(1, max_steering_angle_);
      vel_steer_space->setBounds(vs_bounds);

      auto state_space = se2_space + vel_steer_space;
      // Match DynamicCarPlanning's subspace weights: SE2=1.0, vel/steer=0.3
      state_space->as<ob::CompoundStateSpace>()->setSubspaceWeight(1, 0.0);

      // 2) Build control space: (acceleration, steering_rate)
      auto ctrl_space = std::make_shared<oc::RealVectorControlSpace>(state_space, 2);
      ob::RealVectorBounds ctrl_bounds(2);
      ctrl_bounds.setLow(0, -max_acceleration_);
      ctrl_bounds.setHigh(0, max_acceleration_);
      ctrl_bounds.setLow(1, -max_steering_rate_);
      ctrl_bounds.setHigh(1, max_steering_rate_);
      ctrl_space->setBounds(ctrl_bounds);

      // 3) Control-based SimpleSetup
      oc::SimpleSetup ss(ctrl_space);

      // 4) Define dynamic car ODE
      //    State vector: [x, y, theta, v, phi]
      //    Controls: [acceleration, steering_rate]
      double L = wheelbase_;
      auto ode = [L](const oc::ODESolver::StateType &q,
                     const oc::Control *control,
                     oc::ODESolver::StateType &qdot) {
        const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
        double theta = q[2];
        double v     = q[3];
        double phi   = q[4];
        qdot.resize(q.size(), 0.0);
        qdot[0] = v * std::cos(theta);        // dx
        qdot[1] = v * std::sin(theta);        // dy
        qdot[2] = (v / L) * std::tan(phi);    // dtheta
        qdot[3] = u[0];                       // dv (acceleration)
        qdot[4] = u[1];                       // dphi (steering rate)
      };

      // 5) Create ODE solver, post-propagation to enforce bounds (normalizes SO2 + clamps v, phi)
      auto odeSolver = std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), ode);
      auto postProp = [state_space](const ob::State *, const oc::Control *, double, ob::State *result) {
        state_space->enforceBounds(result);
      };
      ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, postProp));

      // 6) Occupancy-grid validity checker (uses /map topic data)
      ss.setStateValidityChecker(
          [this](const ob::State *state) -> bool {
            const auto *compound = state->as<ob::CompoundState>();
            const auto *se2 = compound->as<ob::SE2StateSpace::StateType>(0);
            Eigen::Vector2d pos(se2->getX(), se2->getY());
            return is_free_space(pos);
          });

      // 7) Propagation parameters
      auto si = ss.getSpaceInformation();
      si->setPropagationStepSize(propagation_step_size_);
      if (planner_type_ == "rcs") {
        // RCS requires fixed control duration
        si->setMinMaxControlDuration(control_duration_steps_, control_duration_steps_);
      } else {
        si->setMinMaxControlDuration(1, control_duration_steps_);
      }

      // 8) Start state (x, y, yaw, v=0, phi=0)
      double start_yaw = yaw_from_quaternion_(start.pose.orientation);
      ob::ScopedState<ob::CompoundStateSpace> s(state_space);
      s->as<ob::SE2StateSpace::StateType>(0)->setX(start.pose.position.x);
      s->as<ob::SE2StateSpace::StateType>(0)->setY(start.pose.position.y);
      s->as<ob::SE2StateSpace::StateType>(0)->setYaw(start_yaw);
      s->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
      s->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = 0.0;

      // 9) Goal state (x, y, yaw, v=0, phi=0)
      double goal_yaw = yaw_from_quaternion_(goal.pose.orientation);
      ob::ScopedState<ob::CompoundStateSpace> g(state_space);
      g->as<ob::SE2StateSpace::StateType>(0)->setX(goal.pose.position.x);
      g->as<ob::SE2StateSpace::StateType>(0)->setY(goal.pose.position.y);
      g->as<ob::SE2StateSpace::StateType>(0)->setYaw(goal_yaw);
      g->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
      g->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = 0.0;
      ss.setStartAndGoalStates(s, g, goal_tolerance_);

      // 10) Planner selection
      if (planner_type_ == "rcs") {
#ifdef HAS_RCS
        auto rcs = std::make_shared<RCSStarPlanner>(si);
        rcs->setHyperparameters(
            rcs_min_resolution_, rcs_speed_cutoff_, rcs_lookahead_,
            rcs_node_cost_factor_, rcs_coarse_control_factor_,
            rcs_discrete_level_, rcs_epsilon_, /*prunF=*/1.0,
            rcs_ics_reject_radius_);
        rcs->setOptimize("LENGTH");
        rcs->setRefinementMode(RCSStarPlanner::RefinementMode::TIME_DOMAIN);
        ss.setPlanner(rcs);
        RCLCPP_INFO(this->get_logger(), "Using RCSStar planner");
#else
        RCLCPP_ERROR(this->get_logger(), "RCS planner requested but not compiled with RCS support");
        return;
#endif
      } else {
        ss.setPlanner(std::make_shared<oc::RRT>(si));
        RCLCPP_INFO(this->get_logger(), "Using kinodynamic RRT planner");
      }

      // 11) Setup and solve
      ss.setup();
      ob::PlannerStatus solved = ss.solve(planning_timeout_sec_);
      if (solved != ob::PlannerStatus::EXACT_SOLUTION) {
        RCLCPP_WARN(this->get_logger(), "No exact solution found in %.2fs (%s).",
                    planning_timeout_sec_, solved.asString().c_str());
        return;
      }

      // 12) Extract path (one state per propagation step, not per control segment) and publish
      oc::PathControl &path = ss.getSolutionPath();
      path.interpolate();

      std::vector<std::array<double, 3>> poses;
      for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        const auto *compound = path.getState(i)->as<ob::CompoundState>();
        const auto *se2 = compound->as<ob::SE2StateSpace::StateType>(0);
        poses.push_back({se2->getX(), se2->getY(), se2->getYaw()});
      }
      publish_path_(poses);
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
      inflate_obstacles_();
      RCLCPP_INFO(this->get_logger(), "Map updated... Ready to plan");
    }

    void inflate_obstacles_() {
      inflated_grid_ = occupancy_grid_;
      int r_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));
      RCLCPP_INFO(this->get_logger(), "Inflating obstacles with radius %.2f m (%d cells)", robot_radius_, r_cells);

      int rows = static_cast<int>(occupancy_grid_.rows());
      int cols = static_cast<int>(occupancy_grid_.cols());
      int r2 = r_cells * r_cells;

      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          if (occupancy_grid_(r, c) >= occupancy_threshold_) {
            int r_min = std::max(0, r - r_cells);
            int r_max = std::min(rows - 1, r + r_cells);
            int c_min = std::max(0, c - r_cells);
            int c_max = std::min(cols - 1, c + r_cells);
            for (int rr = r_min; rr <= r_max; ++rr) {
              for (int cc = c_min; cc <= c_max; ++cc) {
                if ((rr - r) * (rr - r) + (cc - c) * (cc - c) <= r2) {
                  inflated_grid_(rr, cc) = 100;
                }
              }
            }
          }
        }
      }
    }

    void map_callback_(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      save_map(msg->info, msg->data);
    }

    void run_rcs_sweep_(const geometry_msgs::msg::PoseStamped &goal) {
#ifdef HAS_RCS
      RCLCPP_INFO(this->get_logger(), "Starting RCS parameter sweep from config: %s", rcs_sweep_config_path_.c_str());

      YAML::Node config;
      try {
        config = YAML::LoadFile(rcs_sweep_config_path_);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load sweep config: %s", e.what());
        return;
      }

      auto sweep = config["rcs_sweep"];
      std::string output_csv = sweep["output_csv"].as<std::string>("/tmp/rcs_sweep.csv");
      double sweep_timeout = sweep["planning_timeout_sec"].as<double>(30.0);
      int repeat = sweep["repeat"].as<int>(1);

      // Read base vehicle params (fall back to node params)
      auto base = sweep["base"];
      double sw_wheelbase = base["wheelbase"].as<double>(wheelbase_);
      double sw_max_vel = base["max_velocity"].as<double>(max_velocity_);
      double sw_max_steer = base["max_steering_angle"].as<double>(max_steering_angle_);
      double sw_max_accel = base["max_acceleration"].as<double>(max_acceleration_);
      double sw_max_steer_rate = base["max_steering_rate"].as<double>(max_steering_rate_);
      double sw_prop_step = base["propagation_step_size"].as<double>(propagation_step_size_);
      int sw_ctrl_dur = base["control_duration_steps"].as<int>(control_duration_steps_);

      // Read grid arrays
      auto grid = sweep["grid"];
      std::map<std::string, std::vector<double>> grid_values;
      std::vector<std::string> grid_keys;
      for (auto it = grid.begin(); it != grid.end(); ++it) {
        std::string key = it->first.as<std::string>();
        std::vector<double> vals;
        for (const auto &v : it->second) {
          vals.push_back(v.as<double>());
        }
        grid_keys.push_back(key);
        grid_values[key] = vals;
      }

      // Build Cartesian product of all grid arrays
      std::vector<std::map<std::string, double>> combos;
      combos.push_back({});
      for (const auto &key : grid_keys) {
        std::vector<std::map<std::string, double>> new_combos;
        for (const auto &combo : combos) {
          for (double val : grid_values[key]) {
            auto c = combo;
            c[key] = val;
            new_combos.push_back(c);
          }
        }
        combos = std::move(new_combos);
      }

      RCLCPP_INFO(this->get_logger(), "Sweep: %zu combinations x %d repeats = %zu total runs",
                  combos.size(), repeat, combos.size() * repeat);

      // Start/goal from current odom and received goal pose
      geometry_msgs::msg::PoseWithCovariance start = current_odom_->pose;
      double start_yaw = yaw_from_quaternion_(start.pose.orientation);
      double goal_yaw = yaw_from_quaternion_(goal.pose.orientation);

      // Open CSV
      std::ofstream csv(output_csv);
      if (!csv.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open CSV: %s", output_csv.c_str());
        return;
      }
      csv << "run";
      for (const auto &key : grid_keys) csv << "," << key;
      csv << ",solved,time_sec,path_states\n";

      int run_idx = 0;
      int solved_count = 0;

      for (const auto &combo : combos) {
        for (int rep = 0; rep < repeat; ++rep) {
          // Extract hyperparams with defaults
          auto get = [&](const std::string &k, double def) -> double {
            auto it = combo.find(k);
            return (it != combo.end()) ? it->second : def;
          };
          int p_min_res = static_cast<int>(get("rcs_min_resolution", rcs_min_resolution_));
          int p_disc_lev = static_cast<int>(get("rcs_discrete_level", rcs_discrete_level_));
          int p_lookahead = static_cast<int>(get("rcs_lookahead", rcs_lookahead_));
          int p_node_cost = static_cast<int>(get("rcs_node_cost_factor", rcs_node_cost_factor_));
          int p_coarse = static_cast<int>(get("rcs_coarse_control_factor", rcs_coarse_control_factor_));
          double p_epsilon = get("rcs_epsilon", rcs_epsilon_);
          double p_speed_cut = get("rcs_speed_cutoff", rcs_speed_cutoff_);
          double p_ics_rej = get("rcs_ics_reject_radius", rcs_ics_reject_radius_);

          // Build state space
          auto se2_sp = std::make_shared<ob::SE2StateSpace>();
          ob::RealVectorBounds se2_b(2);
          se2_b.setLow(0, origin_[0]);
          se2_b.setLow(1, origin_[1]);
          se2_b.setHigh(0, origin_[0] + width_ * resolution_);
          se2_b.setHigh(1, origin_[1] + height_ * resolution_);
          se2_sp->setBounds(se2_b);

          auto vs_sp = std::make_shared<ob::RealVectorStateSpace>(2);
          ob::RealVectorBounds vs_b(2);
          vs_b.setLow(0, 0.0);
          vs_b.setHigh(0, sw_max_vel);
          vs_b.setLow(1, -sw_max_steer);
          vs_b.setHigh(1, sw_max_steer);
          vs_sp->setBounds(vs_b);

          auto sspace = se2_sp + vs_sp;
          sspace->as<ob::CompoundStateSpace>()->setSubspaceWeight(1, 0.0);

          auto cspace = std::make_shared<oc::RealVectorControlSpace>(sspace, 2);
          ob::RealVectorBounds cb(2);
          cb.setLow(0, -sw_max_accel);
          cb.setHigh(0, sw_max_accel);
          cb.setLow(1, -sw_max_steer_rate);
          cb.setHigh(1, sw_max_steer_rate);
          cspace->setBounds(cb);

          oc::SimpleSetup ss(cspace);

          double L = sw_wheelbase;
          auto ode = [L](const oc::ODESolver::StateType &q,
                         const oc::Control *control,
                         oc::ODESolver::StateType &qdot) {
            const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
            double theta = q[2], v = q[3], phi = q[4];
            qdot.resize(q.size(), 0.0);
            qdot[0] = v * std::cos(theta);
            qdot[1] = v * std::sin(theta);
            qdot[2] = (v / L) * std::tan(phi);
            qdot[3] = u[0];
            qdot[4] = u[1];
          };

          auto solver = std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), ode);
          auto pp = [sspace](const ob::State *, const oc::Control *, double, ob::State *result) {
            sspace->enforceBounds(result);
          };
          ss.setStatePropagator(oc::ODESolver::getStatePropagator(solver, pp));

          ss.setStateValidityChecker(
              [this](const ob::State *state) -> bool {
                const auto *compound = state->as<ob::CompoundState>();
                const auto *se2 = compound->as<ob::SE2StateSpace::StateType>(0);
                return is_free_space(Eigen::Vector2d(se2->getX(), se2->getY()));
              });

          auto si = ss.getSpaceInformation();
          si->setPropagationStepSize(sw_prop_step);
          si->setMinMaxControlDuration(sw_ctrl_dur, sw_ctrl_dur);

          ob::ScopedState<ob::CompoundStateSpace> s_state(sspace);
          s_state->as<ob::SE2StateSpace::StateType>(0)->setX(start.pose.position.x);
          s_state->as<ob::SE2StateSpace::StateType>(0)->setY(start.pose.position.y);
          s_state->as<ob::SE2StateSpace::StateType>(0)->setYaw(start_yaw);
          s_state->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
          s_state->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = 0.0;

          ob::ScopedState<ob::CompoundStateSpace> g_state(sspace);
          g_state->as<ob::SE2StateSpace::StateType>(0)->setX(goal.pose.position.x);
          g_state->as<ob::SE2StateSpace::StateType>(0)->setY(goal.pose.position.y);
          g_state->as<ob::SE2StateSpace::StateType>(0)->setYaw(goal_yaw);
          g_state->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0.0;
          g_state->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = 0.0;

          ss.setStartAndGoalStates(s_state, g_state, rcs_goal_tolerance_);

          auto rcs = std::make_shared<RCSStarPlanner>(si);
          rcs->setHyperparameters(p_min_res, p_speed_cut, p_lookahead,
                                  p_node_cost, p_coarse, p_disc_lev,
                                  p_epsilon, 1.0, p_ics_rej);
          rcs->setOptimize("LENGTH");
          rcs->setRefinementMode(RCSStarPlanner::RefinementMode::TIME_DOMAIN);
          ss.setPlanner(rcs);
          ss.setup();

          auto t0 = std::chrono::steady_clock::now();
          ob::PlannerStatus solved = ss.solve(sweep_timeout);
          auto t1 = std::chrono::steady_clock::now();
          double elapsed = std::chrono::duration<double>(t1 - t0).count();

          std::size_t path_states = 0;
          if (solved) {
            path_states = ss.getSolutionPath().getStateCount();
            solved_count++;
          }

          csv << run_idx;
          for (const auto &key : grid_keys) csv << "," << combo.at(key);
          csv << "," << (solved ? 1 : 0) << "," << elapsed << "," << path_states << "\n";

          RCLCPP_INFO(this->get_logger(), "Sweep run %d: %s in %.2fs (%zu states)",
                      run_idx, solved ? "SOLVED" : "FAILED", elapsed, path_states);
          run_idx++;
        }
      }

      csv.close();
      RCLCPP_INFO(this->get_logger(), "Sweep complete: %d/%d solved. Results at %s",
                  solved_count, run_idx, output_csv.c_str());
#else
      (void)goal;
      RCLCPP_ERROR(this->get_logger(), "RCS sweep requested but not compiled with RCS support");
#endif
    }

    void goal_pose_callback_(const geometry_msgs::msg::PoseStamped &msg) {
      goal_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(msg);
      if (!ready_to_plan_()) {
        return;
      }
      if (rcs_sweep_enabled_ && planner_type_ == "rcs") {
        RCLCPP_INFO(this->get_logger(), "Received new goal pose. Starting RCS sweep");
        run_rcs_sweep_(msg);
      } else if (planner_type_ == "dubins") {
        RCLCPP_INFO(this->get_logger(), "Received new goal pose. Starting to plan");
        plan_dubins_path_(msg);
      } else {
        RCLCPP_INFO(this->get_logger(), "Received new goal pose. Starting to plan");
        plan_path_(msg);
      }
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
      if (inflated_grid_.size() == 0 || width_ == 0 || height_ == 0) {
        RCLCPP_WARN(this->get_logger(), "is_free_space: occupancy grid not initialized yet");
        return false;
      }

      double dx = point[0] - origin_[0];
      double dy = point[1] - origin_[1];

      int col = static_cast<int>(std::floor(dx / resolution_));
      int row = static_cast<int>(std::floor(dy / resolution_));

      if (row < 0 || row >= inflated_grid_.rows() || col < 0 || col >= inflated_grid_.cols()) {
        RCLCPP_DEBUG(this->get_logger(), "is_free_space: point (%.3f, %.3f) -> idx (r=%d,c=%d) out of bounds",
                      point[0], point[1], row, col);
        return false;
      }

      int8_t cell = inflated_grid_(row, col);

      if (cell < 0) return unknown_is_free_;
      if (cell >= occupancy_threshold_) return false;
      return true;
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
      this->declare_parameter("robot_radius", 0.3);
      this->declare_parameter("occupancy_threshold", 50);
      this->declare_parameter("unknown_is_free", false);
      // "dubins": geometric RRT* on Dubins curves (default); "rrt": kinodynamic RRT; "rcs": RCSStar
      this->declare_parameter("planner_type", std::string("dubins"));
      this->declare_parameter("planning_timeout_sec", 1.0);
      this->declare_parameter("goal_tolerance", 0.25);
      this->declare_parameter("min_turning_radius", 1.0);
      this->declare_parameter("path_resolution", 0.1);
      this->declare_parameter("rcs_min_resolution", 0);
      this->declare_parameter("rcs_discrete_level", 2);
      this->declare_parameter("rcs_lookahead", 0);
      this->declare_parameter("rcs_node_cost_factor", 1);
      this->declare_parameter("rcs_coarse_control_factor", 0);
      this->declare_parameter("rcs_ics_reject_radius", 0.0);
      this->declare_parameter("rcs_speed_cutoff", 1e-10);
      this->declare_parameter("rcs_epsilon", 0.5);
      this->declare_parameter("rcs_goal_tolerance", 1.5);
      this->declare_parameter("rcs_sweep.enabled", false);
      this->declare_parameter("rcs_sweep.config_path", std::string(""));
      this->declare_parameter("wheelbase", 0.2032);
      this->declare_parameter("max_velocity", 4.0);
      this->declare_parameter("max_steering_angle", 0.52);
      this->declare_parameter("max_acceleration", 0.5);
      this->declare_parameter("max_steering_rate", 0.035);
      this->declare_parameter("propagation_step_size", 0.3);
      this->declare_parameter("control_duration_steps", 16);

      robot_radius_ = this->get_parameter("robot_radius").as_double();
      occupancy_threshold_ = static_cast<int8_t>(this->get_parameter("occupancy_threshold").as_int());
      unknown_is_free_ = this->get_parameter("unknown_is_free").as_bool();
      planner_type_ = this->get_parameter("planner_type").as_string();
      if (planner_type_ == "rrtstar") {
        // Old name: this was always kinodynamic RRT, never RRT*
        RCLCPP_WARN(this->get_logger(), "planner_type 'rrtstar' is now 'rrt' (kinodynamic RRT)");
        planner_type_ = "rrt";
      }
      planning_timeout_sec_ = this->get_parameter("planning_timeout_sec").as_double();
      goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
      min_turning_radius_ = this->get_parameter("min_turning_radius").as_double();
      path_resolution_ = this->get_parameter("path_resolution").as_double();
      rcs_min_resolution_ = this->get_parameter("rcs_min_resolution").as_int();
      rcs_discrete_level_ = this->get_parameter("rcs_discrete_level").as_int();
      rcs_lookahead_ = this->get_parameter("rcs_lookahead").as_int();
      rcs_node_cost_factor_ = this->get_parameter("rcs_node_cost_factor").as_int();
      rcs_coarse_control_factor_ = this->get_parameter("rcs_coarse_control_factor").as_int();
      rcs_ics_reject_radius_ = this->get_parameter("rcs_ics_reject_radius").as_double();
      rcs_speed_cutoff_ = this->get_parameter("rcs_speed_cutoff").as_double();
      rcs_epsilon_ = this->get_parameter("rcs_epsilon").as_double();
      rcs_goal_tolerance_ = this->get_parameter("rcs_goal_tolerance").as_double();
      rcs_sweep_enabled_ = this->get_parameter("rcs_sweep.enabled").as_bool();
      rcs_sweep_config_path_ = this->get_parameter("rcs_sweep.config_path").as_string();
      wheelbase_ = this->get_parameter("wheelbase").as_double();
      max_velocity_ = this->get_parameter("max_velocity").as_double();
      max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
      max_acceleration_ = this->get_parameter("max_acceleration").as_double();
      max_steering_rate_ = this->get_parameter("max_steering_rate").as_double();
      propagation_step_size_ = this->get_parameter("propagation_step_size").as_double();
      control_duration_steps_ = this->get_parameter("control_duration_steps").as_int();

      // Topic names (bringup resolves these per mode; defaults are the sim names)
      const auto odom_topic = this->declare_parameter("odom_topic", std::string("/ego_racecar/odom"));
      const auto map_topic = this->declare_parameter("map_topic", std::string("/map"));
      const auto goal_pose_topic = this->declare_parameter("goal_pose_topic", std::string("/goal_pose"));
      const auto clicked_point_topic = this->declare_parameter("clicked_point_topic", std::string("/clicked_point"));
      const auto planned_path_topic = this->declare_parameter("planned_path_topic", std::string("/planned_path"));
      const auto map_server = this->declare_parameter("map_server", std::string("/map_server/map"));

      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, 10, std::bind(&PlanningNode::odom_callback_, this, std::placeholders::_1));
      // Latched: map_server and mocap_map_node both publish the map transient_local
      const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          map_topic, latched_qos, std::bind(&PlanningNode::map_callback_, this, std::placeholders::_1));

      goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_pose_topic, 10, std::bind(&PlanningNode::goal_pose_callback_, this, std::placeholders::_1));

      clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        clicked_point_topic, 10, std::bind(&PlanningNode::clicked_point_callback_, this, std::placeholders::_1));

      map_client_ = this->create_client<nav_msgs::srv::GetMap>(map_server);

      // Try until service is up, then request once
      map_request_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PlanningNode::try_request_map_, this)
      );


      // Latched to match pure_pursuit_node's transient_local path subscription
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>(planned_path_topic, latched_qos);


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
