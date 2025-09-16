#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <chrono>
#include <atomic>
#include <thread>
#include <functional>
#include <cmath>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

using namespace std::chrono_literals;

class CartesianPickPlace : public rclcpp::Node
{
public:
  struct Sextet {
    geometry_msgs::msg::Pose pre_pick;
    geometry_msgs::msg::Pose pick;
    geometry_msgs::msg::Pose post_pick;
    geometry_msgs::msg::Pose pre_place;
    geometry_msgs::msg::Pose place;
    geometry_msgs::msg::Pose post_place;
  };

  explicit CartesianPickPlace(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("cartesian_pick_place", options)
  {
    // ---- Parameters (trajectory & scaling)
    robot_name_   = get_parameter("robot_name").as_string();
    eef_step_     = this->declare_parameter<double>("eef_step",   0.005);
    jump_thresh_  = this->declare_parameter<double>("jump_threshold", 20.0);
    vel_scale_    = this->declare_parameter<double>("vel_scale",  0.2);
    acc_scale_    = this->declare_parameter<double>("acc_scale",  0.2);
    time_scale_   = this->declare_parameter<double>("time_scale", 1.5);

    // ---- Parameters (target interpretation)
    units_to_m_   = this->declare_parameter<double>("units_to_m", 0.01);   // 1 field unit = 1 cm
    //y_offset_m_   = this->declare_parameter<double>("y_offset_m", 0.32);   // keep as provided // left arm
    y_offset_m_   = this->declare_parameter<double>("y_offset_m", 0.28);   // keep as provided // right arm
    y_goal_tol_m_ = this->declare_parameter<double>("y_goal_tol_m", 0.01); // same-target tolerance
    valid_min_    = this->declare_parameter<double>("valid_min", 0.0);
    valid_max_    = this->declare_parameter<double>("valid_max", 60.0);

    // “Already at home” positional tolerance
    home_pos_tol_m_ = this->declare_parameter<double>("home_pos_tol_m", 0.015);

    // ---- Parameters (pick/place geometry)
    pre_pick_x_m_     = this->declare_parameter<double>("pre_pick_x_m", 0.7006);
    pick_x_m_         = this->declare_parameter<double>("pick_x_m",     0.7506);
    //pick_z_grasp_m_   = this->declare_parameter<double>("pick_z_grasp_m", 1.015); // left arm
    pick_z_grasp_m_   = this->declare_parameter<double>("pick_z_grasp_m", 0.9855); // right arm
    // pre_pick_z_m_     = this->declare_parameter<double>("pre_pick_z_m",   1.083); // left arm
    pre_pick_z_m_     = this->declare_parameter<double>("pre_pick_z_m",   1.053); // right arm

    pre_place_x_m_    = this->declare_parameter<double>("pre_place_x_m", 0.7006);
    place_x_m_        = this->declare_parameter<double>("place_x_m",     0.7506);
    //place_y_m_        = this->declare_parameter<double>("place_y_m",     0.5860); // left arm
    place_y_m_        = this->declare_parameter<double>("place_y_m",     -0.6513); // right arm
    //place_z_m_        = this->declare_parameter<double>("place_z_m",     1.130); // left arm
    place_z_m_        = this->declare_parameter<double>("place_z_m",     1.10);
    //pre_place_z_m_    = this->declare_parameter<double>("pre_place_z_m", 1.203); // left arm
    pre_place_z_m_    = this->declare_parameter<double>("pre_place_z_m", 1.170);

    // Home (invalid target → go here)
    //home_pose_ = makePose(0.520, 0.226, 1.082, 0.929, -0.027, 0.369, 0.006); // left arm
    home_pose_ = makePose(0.363, -0.517, 0.451, 0.115, -0.557, 0.701, -0.430); // right arm

    // ---- IO
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/onrobot/finger_width_controller/commands", 10);

    sub_target_pos_ = this->create_subscription<std_msgs::msg::Float64>(
      "target_position", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&CartesianPickPlace::targetPosCallback, this, std::placeholders::_1));

    sub_restart_ = this->create_subscription<std_msgs::msg::Bool>(
      "task_restart", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&CartesianPickPlace::restartCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "CartesianPickPlace ready. Subscribed to 'target_position'.");

    // Startup: open gripper and (optionally) go HOME (non-preemptive)
    startup_timer_ = this->create_wall_timer(300ms, [this]() {
      startup_timer_->cancel();
      gripper_open();
      std::this_thread::sleep_for(1s);
      go_to_home_pose(/*preempt=*/false);
    });
  }

  // Call this right after you make_shared<> the node
  void init(const rclcpp::Node::SharedPtr & self)
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      self,
      moveit::planning_interface::MoveGroupInterface::Options(
        "arm", "robot_description", robot_name_));
    move_group_->setStartStateToCurrentState();
    move_group_->setGoalPositionTolerance(2e-3);
    move_group_->setGoalOrientationTolerance(5e-2);
    move_group_->setMaxVelocityScalingFactor(vel_scale_);
    move_group_->setMaxAccelerationScalingFactor(acc_scale_);

    // Joint constraints (from your working node)
    moveit_msgs::msg::Constraints path_constraints;
    moveit_msgs::msg::JointConstraint jc;

    jc.joint_name = "lbr_A1"; jc.position = (-95.0)*M_PI/180.0; jc.tolerance_above = (45.0)*M_PI/180.0; jc.tolerance_below = (45.0)*M_PI/180.0; jc.weight = 1.0; path_constraints.joint_constraints.push_back(jc);
    jc.joint_name = "lbr_A2"; jc.position = (-85.0)*M_PI/180.0; jc.tolerance_above = (30.0)*M_PI/180.0; jc.tolerance_below = (30.0)*M_PI/180.0; jc.weight = 1.0; path_constraints.joint_constraints.push_back(jc);
    jc.joint_name = "lbr_A3"; jc.position = ( 60.0)*M_PI/180.0; jc.tolerance_above = (20.0)*M_PI/180.0; jc.tolerance_below = (20.0)*M_PI/180.0; jc.weight = 1.0; path_constraints.joint_constraints.push_back(jc);
    jc.joint_name = "lbr_A4"; jc.position = (-65.0)*M_PI/180.0; jc.tolerance_above = (45.0)*M_PI/180.0; jc.tolerance_below = (45.0)*M_PI/180.0; jc.weight = 1.0; path_constraints.joint_constraints.push_back(jc);
    jc.joint_name = "lbr_A5"; jc.position = ( 10.0)*M_PI/180.0; jc.tolerance_above = (60.0)*M_PI/180.0; jc.tolerance_below = (60.0)*M_PI/180.0; jc.weight = 1.0; path_constraints.joint_constraints.push_back(jc);
    jc.joint_name = "lbr_A6"; jc.position = (-40.0)*M_PI/180.0; jc.tolerance_above = (20.0)*M_PI/180.0; jc.tolerance_below = (20.0)*M_PI/180.0; jc.weight = 1.0; path_constraints.joint_constraints.push_back(jc);
    jc.joint_name = "lbr_A7"; jc.position = (-12.5)*M_PI/180.0; jc.tolerance_above = (47.5)*M_PI/180.0; jc.tolerance_below = (47.5)*M_PI/180.0; jc.weight = 1.0; path_constraints.joint_constraints.push_back(jc);

    move_group_->setPathConstraints(path_constraints);
    RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
  }

  ~CartesianPickPlace() override
  {
    request_cancel_ = true;
    if (sequence_thread_.joinable()) {
      try { move_group_->stop(); } catch (...) {}
      sequence_thread_.join();
    }
    if (kick_timer_) kick_timer_->cancel();
  }

private:
  // ---------- Utilities ----------
  static geometry_msgs::msg::Pose makePose(double px,double py,double pz,
                                           double ox,double oy,double oz,double ow)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = px; p.position.y = py; p.position.z = pz;
    p.orientation.x = ox; p.orientation.y = oy; p.orientation.z = oz; p.orientation.w = ow;
    return p;
  }

  static geometry_msgs::msg::Pose makePoseXYZ(double x, double y, double z)
  {
    geometry_msgs::msg::Pose p;
    p.position.x = x; p.position.y = y; p.position.z = z;
    p.orientation.w = 1.0; // orientation will be overwritten to current in planner
    return p;
  }

  bool isValidUnits(double u) const
  {
    return std::isfinite(u) && u >= valid_min_ && u <= valid_max_ && (u != 100.0);
  }

  Sextet make_sextet_for_y(double y_m) const
  {
    Sextet s;
    s.pre_pick  = makePoseXYZ(pre_pick_x_m_, y_m, pre_pick_z_m_);
    s.pick      = makePoseXYZ(pick_x_m_,     y_m, pick_z_grasp_m_);
    s.post_pick = makePoseXYZ(pre_pick_x_m_, y_m, pre_pick_z_m_);
    s.pre_place = makePoseXYZ(pre_place_x_m_, place_y_m_, pre_place_z_m_);
    s.place     = makePoseXYZ(place_x_m_,     place_y_m_, place_z_m_);
    s.post_place= makePoseXYZ(pre_place_x_m_, place_y_m_, pre_place_z_m_);
    return s;
  }

  // ---------- Gripper ----------
  void gripper_set(double width)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = { width };
    gripper_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Gripper -> width=%.3f", width);
  }
  void gripper_open()  { gripper_set(0.08); }
  void gripper_close() { gripper_set(0.0385); }

  // ---------- Quick checks ----------
  bool isAtHome() const
  {
    // // // Current pose vs. home position (ignoring orientation)
    // move_group_->setStartStateToCurrentState();
    // const auto cur = move_group_->getCurrentPose().pose;
    // const auto &h  = home_pose_;
    // const double dx = cur.position.x - h.position.x;
    // const double dy = cur.position.y - h.position.y;
    // const double dz = cur.position.z - h.position.z;
    // const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    // return dist <= home_pos_tol_m_;
    return going_home_;
  }

  // ---------- Planning & execution (Cartesian with timing) ----------
  bool plan_and_execute_cartesian_(const geometry_msgs::msg::Pose &target,
                                 const std::string &label)
  {
    if (request_cancel_) return false;
    going_home_ = false;

    move_group_->setStartStateToCurrentState();
    const auto start_pose = move_group_->getCurrentPose().pose;

    auto corrected_target = target;
    corrected_target.orientation = start_pose.orientation; // lock to current

    std::vector<geometry_msgs::msg::Pose> waypoints{ start_pose, corrected_target };

    // --- Helper: validate A2 across all points in degrees
    auto a2_within_115deg = [&](const moveit_msgs::msg::RobotTrajectory& plan_traj) -> bool {
      const auto& jt = plan_traj.joint_trajectory;
      auto it = std::find(jt.joint_names.begin(), jt.joint_names.end(), "lbr_A2");
      if (it == jt.joint_names.end() || jt.points.empty()) return true; // nothing to check
      const size_t idx = std::distance(jt.joint_names.begin(), it);

      for (size_t i = 0; i < jt.points.size(); ++i) {
        const auto& pt = jt.points[i];
        if (idx >= pt.positions.size()) continue;
        const double a2_deg = pt.positions[idx] * 180.0 / M_PI;
        if (std::fabs(a2_deg) > 115.0) {
          RCLCPP_WARN(get_logger(),
                      "[%s] A2 out of bounds at point %zu: %.2f deg (limit ±115).",
                      label.c_str(), i, a2_deg);
          return false;
        }
      }
      return true;
    };

    // --- Helper: build a time-parameterized plan from a raw trajectory
    auto build_timed_plan = [&](const moveit_msgs::msg::RobotTrajectory& raw_traj,
                                moveit::planning_interface::MoveGroupInterface::Plan& out_plan) -> bool {
      auto current_state = move_group_->getCurrentState(0.5);
      if (!current_state) {
        RCLCPP_WARN(get_logger(), "No current state for time parameterization; executing raw trajectory.");
        out_plan.trajectory_ = raw_traj;
        return true;
      }
      robot_trajectory::RobotTrajectory rt(current_state->getRobotModel(), move_group_->getName());
      rt.setRobotTrajectoryMsg(*current_state, raw_traj);

      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      bool timed_ok = totg.computeTimeStamps(rt, vel_scale_, acc_scale_);
      if (!timed_ok) {
        RCLCPP_WARN(get_logger(), "Time parameterization failed; using raw trajectory.");
        out_plan.trajectory_ = raw_traj;
      } else {
        rt.getRobotTrajectoryMsg(out_plan.trajectory_);
        if (time_scale_ != 1.0) {
          auto &pts = out_plan.trajectory_.joint_trajectory.points;
          for (auto &pt : pts) {
            double t = static_cast<double>(pt.time_from_start.sec)
                    + static_cast<double>(pt.time_from_start.nanosec) * 1e-9;
            t *= time_scale_;
            builtin_interfaces::msg::Duration dur;
            dur.sec = static_cast<int32_t>(std::floor(t));
            dur.nanosec = static_cast<uint32_t>(std::round((t - dur.sec) * 1e9));
            pt.time_from_start = dur;
            if (!pt.velocities.empty())
              for (auto &v : pt.velocities) v /= time_scale_;
            if (!pt.accelerations.empty())
              for (auto &a : pt.accelerations) a /= (time_scale_ * time_scale_);
          }
        }
      }
      return true;
    };

    // --- Attempt 1: current eef_step_
    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step_, jump_thresh_, traj, /*avoid_collisions=*/true);

    if (request_cancel_) {
      RCLCPP_INFO(get_logger(), "Cancelled before time-parameterization for %s", label.c_str());
      return false;
    }

    if (fraction < 0.90) {
      RCLCPP_ERROR(get_logger(), "Cartesian plan %s only %.1f%% complete (step=%.3f m)",
                  label.c_str(), fraction*100.0, eef_step_);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Cartesian plan %s %.1f%% complete", label.c_str(), fraction*100.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    build_timed_plan(traj, plan);

    // --- Safety check for A2 (±115°). If violated, try a single replan with smaller step.
    if (!a2_within_115deg(plan.trajectory_)) {
      const double retry_step = std::max(1e-4, eef_step_ * 0.5);
      RCLCPP_WARN(get_logger(), "[%s] Replanning with smaller eef_step=%.6f (was %.6f)",
                  label.c_str(), retry_step, eef_step_);

      moveit_msgs::msg::RobotTrajectory traj2;
      double fraction2 = move_group_->computeCartesianPath(
          waypoints, retry_step, jump_thresh_, traj2, /*avoid_collisions=*/true);

      if (fraction2 < 0.90) {
        RCLCPP_ERROR(get_logger(), "Retry plan %s only %.1f%% complete (step=%.6f m)",
                    label.c_str(), fraction2*100.0, retry_step);
        return false;
      }

      build_timed_plan(traj2, plan);

      if (!a2_within_115deg(plan.trajectory_)) {
        RCLCPP_ERROR(get_logger(), "[%s] A2 still exceeds ±115° after retry. Aborting before execution.", label.c_str());
        return false;
      }
    }

    // --- (Optional) Log final target A2 in degrees, before execution
    if (!plan.trajectory_.joint_trajectory.points.empty()) {
      const auto& last = plan.trajectory_.joint_trajectory.points.back();
      const auto& names = plan.trajectory_.joint_trajectory.joint_names;
      auto it = std::find(names.begin(), names.end(), "lbr_A2");
      if (it != names.end()) {
        size_t idx = std::distance(names.begin(), it);
        if (idx < last.positions.size()) {
          double a2_deg = last.positions[idx] * 180.0 / M_PI;
          RCLCPP_INFO(get_logger(), "Target joint A2 = %.2f deg", a2_deg);
        }
      }
    }

    if (request_cancel_) {
      RCLCPP_INFO(get_logger(), "Cancelled before execute for %s", label.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Exec %s (Cartesian) [vel=%.3f acc=%.3f time_scale=%.2f]",
                label.c_str(), vel_scale_, acc_scale_, time_scale_);

    try {
      auto ec = move_group_->execute(plan);
      bool ok = (ec == moveit::core::MoveItErrorCode::SUCCESS);
      if (!ok)
        RCLCPP_ERROR(get_logger(), "%s execution failed (code %d).", label.c_str(), ec.val);
      return ok && !request_cancel_;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception during execute(%s): %s", label.c_str(), e.what());
      return false;
    }
  }


  // ---------- HOME ----------
  void go_to_home_pose(bool preempt)
  {
    auto begin = [this]() {
      // If we're already close enough to home, don't plan again.
      if (isAtHome()) {
        RCLCPP_DEBUG(get_logger(), "Already at HOME (<= %.3f m) — skipping plan.", home_pos_tol_m_);
        {
          std::lock_guard<std::mutex> lk(mutex_);
          stage_ = Stage::NONE;
          busy_ = false;
          request_cancel_ = false;
          active_y_.reset();
        }
        is_homing_.store(false, std::memory_order_relaxed);

        // Handoff to pending request, if any
        if (pending_y_.has_value() || pending_go_home_) {
          auto next_y = pending_y_;
          bool next_home = pending_go_home_;
          pending_y_.reset();
          pending_go_home_ = false;

          busy_ = true;
          kick_timer_ = this->create_wall_timer(0ms, [this, next_y, next_home]() {
            kick_timer_->cancel();
            if (next_home) go_to_home_pose(/*preempt=*/false);
            else if (next_y.has_value()) start_sequence_async_y_(*next_y);
          });
        }
        return;
      }

      cancel_sequence_thread_if_any_();
      request_cancel_ = false;
      busy_ = true;
      stage_ = Stage::NONE;
      active_y_.reset();
      is_homing_.store(true, std::memory_order_relaxed);

      sequence_thread_ = std::thread([this]() {
        bool ok = plan_and_execute_cartesian_(home_pose_, "home");
        if (!ok) {
          RCLCPP_WARN(get_logger(), "Cartesian → home failed; trying joint-space planning fallback.");
          move_group_->setPoseTarget(home_pose_);
          auto ec = move_group_->move();
          ok = (ec == moveit::core::MoveItErrorCode::SUCCESS);
        }
        going_home_ = true;
        std::lock_guard<std::mutex> lk(mutex_);
        stage_ = Stage::NONE;
        busy_ = false;
        request_cancel_ = false;
        active_y_.reset();
        is_homing_.store(false, std::memory_order_relaxed);

        // Handoff to pending request, if any
        if (pending_y_.has_value() || pending_go_home_) {
          auto next_y = pending_y_;
          bool next_home = pending_go_home_;
          pending_y_.reset();
          pending_go_home_ = false;

          busy_ = true;
          kick_timer_ = this->create_wall_timer(0ms, [this, next_y, next_home]() {
            kick_timer_->cancel();
            if (next_home) go_to_home_pose(/*preempt=*/false);
            else if (next_y.has_value()) start_sequence_async_y_(*next_y);
          });
        }
      });
    };

    if (preempt && busy_) {
      RCLCPP_INFO(get_logger(), "Preempting current to go HOME.");
      request_cancel_ = true;
      try { move_group_->stop(); } catch (...) {}
      begin();
    } else {
      begin();
    }
  }

  // ---------- Sequence management ----------
  void start_sequence_async_y_(double y_m)
  {
    cancel_sequence_thread_if_any_();
    request_cancel_ = false;
    active_y_ = y_m;
    stage_ = Stage::PRE_PICK;

    const Sextet steps = make_sextet_for_y(y_m);

    sequence_thread_ = std::thread([this, steps]() {
      // PRE-PICK
      if (request_cancel_) return;
      if (!plan_and_execute_cartesian_(steps.pre_pick, "pre_pick")) { finish_sequence_(false); return; }

      // PICK
      if (request_cancel_) { finish_sequence_(false); return; }
      stage_ = Stage::PICK;
      if (!plan_and_execute_cartesian_(steps.pick, "pick")) { finish_sequence_(false); return; }
      gripper_close();
      std::this_thread::sleep_for(3s);

      // POST-PICK
      stage_ = Stage::POST_PICK;
      if (!plan_and_execute_cartesian_(steps.post_pick, "post_pick")) { finish_sequence_(false); return; }

      // PRE-PLACE
      stage_ = Stage::PRE_PLACE;
      if (!plan_and_execute_cartesian_(steps.pre_place, "pre_place")) { finish_sequence_(false); return; }

      // PLACE
      stage_ = Stage::PLACE;
      if (!plan_and_execute_cartesian_(steps.place, "place")) { finish_sequence_(false); return; }
      gripper_open();
      std::this_thread::sleep_for(2s);

      // POST-PLACE
      stage_ = Stage::POST_PLACE;
      if (!plan_and_execute_cartesian_(steps.post_place, "post_place")) { finish_sequence_(false); return; }

      finish_sequence_(true);
    });
  }

  void finish_sequence_(bool success)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (success) {
      RCLCPP_INFO(get_logger(), "Sequence complete at y=%.3f m.", active_y_.value_or(std::nan("")));
    } else {
      RCLCPP_WARN(get_logger(), "Sequence ended early.");
    }

    stage_ = Stage::NONE;
    busy_  = false;
    request_cancel_ = false;
    active_y_.reset();

    if (pending_y_.has_value() || pending_go_home_) {
      auto next_y = pending_y_;
      bool next_home = pending_go_home_;
      pending_y_.reset();
      pending_go_home_ = false;

      busy_ = true;
      kick_timer_ = this->create_wall_timer(0ms, [this, next_y, next_home]() {
        kick_timer_->cancel();
        if (next_home) go_to_home_pose(/*preempt=*/false);
        else if (next_y.has_value()) start_sequence_async_y_(*next_y);
      });
    }
  }

  void cancel_sequence_thread_if_any_()
  {
    if (!sequence_thread_.joinable()) return;

    if (std::this_thread::get_id() == sequence_thread_.get_id()) {
      request_cancel_ = true;
      try { move_group_->stop(); } catch (...) {}
      return; // let it unwind
    }

    request_cancel_ = true;
    try { move_group_->stop(); } catch (...) {}
    sequence_thread_.join();
  }

  // ---------- Subscribers ----------
  void restartCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      RCLCPP_INFO(get_logger(), "Task restart 'false' — no action.");
      return;
    }
    RCLCPP_WARN(get_logger(), "Task restart received — no per-target memory to clear in continuous mode.");
  }

  void targetPosCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    const double u = msg->data;

    const bool invalid = !isValidUnits(u);
    if (invalid) {
      // Invalid → request HOME once; ignore further invalids while homing or when already there
      if (is_homing_.load(std::memory_order_relaxed)) {
        RCLCPP_DEBUG(get_logger(), "Invalid target (%.3f) while homing — ignored.", u);
        return;
      }
      if (isAtHome()) {
        RCLCPP_DEBUG(get_logger(), "Invalid target (%.3f) but already at HOME — ignored.", u);
        return;
      }

      if (busy_) {
        if (stage_ == Stage::PRE_PICK || !active_y_.has_value()) {
          if (!pending_go_home_) {
            RCLCPP_INFO(get_logger(), "Invalid target (%.3f). Preempt → HOME.", u);
            pending_y_.reset();
            pending_go_home_ = true;
            request_cancel_ = true;
            try { move_group_->stop(); } catch (...) {}
          } else {
            RCLCPP_DEBUG(get_logger(), "HOME already pending — duplicate invalid ignored.");
          }
        } else {
          RCLCPP_INFO(get_logger(), "Invalid target (%.3f) but committed past PRE_PICK; ignoring.", u);
        }
      } else {
        RCLCPP_INFO(get_logger(), "Invalid target (%.3f). Going HOME.", u);
        busy_ = true;
        go_to_home_pose(/*preempt=*/false);
      }
      return;
    }

    // Valid → convert to meters (keep your mapping exactly)
    // +/-0.7332m is starting position to the left and to the right respectively
    // y_offset_m_ is the dimension of the box + an offset to avoid collisions
    //const double y_m = 0.7332 - (60 - (u * 1)) * units_to_m_ - y_offset_m_; // left arm
    const double y_m = -0.7332 + u*units_to_m_ + y_offset_m_; // right arm

    // If already executing same target (within tol), ignore
    if (busy_ && active_y_.has_value() && std::fabs(*active_y_ - y_m) <= y_goal_tol_m_) {
      RCLCPP_DEBUG(get_logger(), "Target y=%.3f m ~ current y=%.3f m; ignoring.", y_m, *active_y_);
      return;
    }

    if (busy_) {
      // If we're homing, allow preempt straight to the valid target
      if (is_homing_.load(std::memory_order_relaxed)) {
        RCLCPP_INFO(get_logger(), "Preempt HOME → y=%.3f m (raw=%.3f).", y_m, u);
        pending_y_ = y_m;
        pending_go_home_ = false;
        request_cancel_ = true;
        try { move_group_->stop(); } catch (...) {}
        return;
      }

      if (!active_y_.has_value() || stage_ == Stage::PRE_PICK) {
        RCLCPP_INFO(get_logger(), "Preempt → y=%.3f m (raw=%.3f).", y_m, u);
        pending_y_ = y_m;
        pending_go_home_ = false;
        request_cancel_ = true;
        try { move_group_->stop(); } catch (...) {}
        RCLCPP_INFO(get_logger(), "Preempt signalled; current execution will abort shortly.");
      } else {
        RCLCPP_INFO(get_logger(),
          "Ignoring new y=%.3f m; already committed (stage=%d).",
          y_m, static_cast<int>(stage_));
      }
      return;
    }

    // Not busy → start immediately
    busy_ = true;
    start_sequence_async_y_(y_m);
  }

private:
  // --- MoveIt
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // --- IO
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_target_pos_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  sub_restart_;
  rclcpp::TimerBase::SharedPtr kick_timer_;

  // --- Home & state
  geometry_msgs::msg::Pose home_pose_;
  std::mutex   mutex_;

  std::string  robot_name_;
  double       eef_step_{0.005};
  double       jump_thresh_{20.0};
  double       vel_scale_{0.2};
  double       acc_scale_{0.2};
  double       time_scale_{1.0};

  // Target interpretation
  double       units_to_m_{0.01};
  double       y_offset_m_{0.32};      // keep as-is
  double       y_goal_tol_m_{0.01};
  double       valid_min_{0.0}, valid_max_{60.0};

  // Geometry
  double pre_pick_x_m_{0.7006}, pick_x_m_{0.7506};
  double pick_z_grasp_m_{1.015}, pre_pick_z_m_{1.083};
  double pre_place_x_m_{0.7006}, place_x_m_{0.7506};
  double place_y_m_{0.5860}, place_z_m_{1.100}, pre_place_z_m_{1.203};

  // “At home” tolerance
  double home_pos_tol_m_{0.015};

  enum class Stage { NONE, PRE_PICK, PICK, POST_PICK, PRE_PLACE, PLACE, POST_PLACE };
  Stage        stage_{Stage::NONE};

  std::atomic<bool> busy_{false};
  std::atomic<bool> request_cancel_{false};
  std::thread       sequence_thread_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  // Active/pending targets (continuous Y)
  std::optional<double> active_y_;
  std::optional<double> pending_y_;
  bool pending_go_home_{false};
  bool going_home_{false};
  std::atomic<bool> is_homing_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<CartesianPickPlace>(options);
  node->init(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
