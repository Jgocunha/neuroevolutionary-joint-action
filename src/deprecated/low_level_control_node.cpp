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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
    // ---- Parameters
    robot_name_   = get_parameter("robot_name").as_string();
    eef_step_     = this->declare_parameter<double>("eef_step",   0.005);  // resolution of waypoints along the Cartesian path
    jump_thresh_  = this->declare_parameter<double>("jump_threshold", 20.0);
    vel_scale_    = this->declare_parameter<double>("vel_scale",  0.2);   // % of max. vel. higher-faster
    acc_scale_    = this->declare_parameter<double>("acc_scale",  0.2);   // % of max. acc. higher-faster
    time_scale_   = this->declare_parameter<double>("time_scale", 1.5);   // 2.0 = twice slower

    // ---- IO
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/onrobot/finger_width_controller/commands", 10);

    sub_target_ = this->create_subscription<std_msgs::msg::Int32>(
      "target_object", 20,
      std::bind(&CartesianPickPlace::targetCallback, this, std::placeholders::_1));

    sub_restart_ = this->create_subscription<std_msgs::msg::Bool>(
      "task_restart", 20,
      std::bind(&CartesianPickPlace::restartCallback, this, std::placeholders::_1));

    // ---- Targets (poses)
    auto P = [this](double px,double py,double pz, double ox,double oy,double oz,double ow){
      geometry_msgs::msg::Pose p;
      p.position.x = px; p.position.y = py; p.position.z = pz;
      p.orientation.x = ox; p.orientation.y = oy; p.orientation.z = oz; p.orientation.w = ow;
      return p;
    };

    // Home (invalid target → go here)
    home_pose_ = P(0.497, 0.226, 1.082, 0.929, -0.027, 0.369, 0.006);

    //(20cm spacing, 60cm span)
    // All share the same nominal orientation, but we lock to current orientation in planning anyway
    const auto qx = 0.761, qy = -0.203, qz = 0.611, qw = 0.079;
    // geometry_msgs::msg::Pose pre_place  =  P(0.7006, 0.5860, 1.103, qx,qy,qz,qw);
    // geometry_msgs::msg::Pose place      =  P(0.7506, 0.5860, 1.060, qx,qy,qz,qw);
    // geometry_msgs::msg::Pose post_place =  P(0.7006, 0.5860, 1.103, qx,qy,qz,qw);

    targets_ = {
      {3, { P(0.7006, 0.3132, 1.083, qx,qy,qz,qw), // pre_pick (left)
            P(0.7506, 0.3132, 1.015, qx,qy,qz,qw), // pick
            P(0.7006, 0.3132, 1.083, qx,qy,qz,qw), // post_pick
            P(0.7006, 0.6460, 1.203, qx,qy,qz,qw), // pre-place
            P(0.7506, 0.6460, 1.100, qx,qy,qz,qw), // place
            P(0.7006, 0.6460, 1.203, qx,qy,qz,qw)  //post-place
          }
        },
      {2, { P(0.7006, 0.1132, 1.083, qx,qy,qz,qw), // pre_pick (center)
            P(0.7506, 0.1132, 1.015, qx,qy,qz,qw), // pick
            P(0.7006, 0.1132, 1.083, qx,qy,qz,qw), // post_pick
            P(0.7006, 0.5860, 1.203, qx,qy,qz,qw), // pre-place
            P(0.7506, 0.5860, 1.100, qx,qy,qz,qw), // place
            P(0.7006, 0.5860, 1.203, qx,qy,qz,qw)  //post-place
          }
        },
      {1, { P(0.7006, -0.0858, 1.083, qx,qy,qz,qw), // pre_pick (right)
            P(0.7506, -0.0858, 1.015, qx,qy,qz,qw), // pick
            P(0.7006, -0.0858, 1.083, qx,qy,qz,qw), // post_pick
            P(0.7006, 0.5260, 1.203, qx,qy,qz,qw), // pre-place
            P(0.7506, 0.5260, 1.100, qx,qy,qz,qw), // place
            P(0.7006, 0.5260, 1.203, qx,qy,qz,qw)  //post-place
          }
      }
    };

    RCLCPP_INFO(get_logger(), "CartesianPickPlace ready. Subscribed to 'target_object'.");

    // Startup: open gripper and (optionally) go home (non-preemptive)
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
    move_group_->setGoalPositionTolerance(2e-3); // from working node
    move_group_->setGoalOrientationTolerance(5e-2); // from working node    
    move_group_->setMaxVelocityScalingFactor(vel_scale_);
    move_group_->setMaxAccelerationScalingFactor(acc_scale_);

    moveit_msgs::msg::Constraints path_constraints;
    moveit_msgs::msg::JointConstraint jc;

    // A1: -140° to -50°
    jc.joint_name = "lbr_A1";
    jc.position = (-95.0) * M_PI / 180.0;
    jc.tolerance_above = (45.0) * M_PI / 180.0;
    jc.tolerance_below = (45.0) * M_PI / 180.0;
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);

    // A2: -115° to -50°
    jc.joint_name = "lbr_A2";
    jc.position = (-82.5) * M_PI / 180.0;
    jc.tolerance_above = (25.5) * M_PI / 180.0;
    jc.tolerance_below = (25.5) * M_PI / 180.0;
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);

    // A3: 40° to 80°
    jc.joint_name = "lbr_A3";
    jc.position = (60.0) * M_PI / 180.0;
    jc.tolerance_above = (20.0) * M_PI / 180.0;
    jc.tolerance_below = (20.0) * M_PI / 180.0;
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);

    // A4: -110° to -20°
    jc.joint_name = "lbr_A4";
    jc.position = (-65.0) * M_PI / 180.0;
    jc.tolerance_above = (45.0) * M_PI / 180.0;
    jc.tolerance_below = (45.0) * M_PI / 180.0;
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);

    // A5: -50° to 70°
    jc.joint_name = "lbr_A5";
    jc.position = (10.0) * M_PI / 180.0;
    jc.tolerance_above = (60.0) * M_PI / 180.0;
    jc.tolerance_below = (60.0) * M_PI / 180.0;
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);

    // A6: -60° to -20°
    jc.joint_name = "lbr_A6";
    jc.position = (-40.0) * M_PI / 180.0;
    jc.tolerance_above = (20.0) * M_PI / 180.0;
    jc.tolerance_below = (20.0) * M_PI / 180.0;
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);

    // A7: -60° to 35°
    jc.joint_name = "lbr_A7";
    jc.position = (-12.5) * M_PI / 180.0;
    jc.tolerance_above = (47.5) * M_PI / 180.0;
    jc.tolerance_below = (47.5) * M_PI / 180.0;
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);

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
    if (kick_timer_) kick_timer_->cancel();  // ensure no timer fires during teardown
  }

private:
  // --- MoveIt
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // --- IO
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_target_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  sub_restart_;
  rclcpp::TimerBase::SharedPtr kick_timer_;  // executor-thread handoff

  // --- Targets and state
  std::map<int, Sextet> targets_;
  geometry_msgs::msg::Pose home_pose_;
  std::set<int> completed_;
  std::mutex   mutex_;

  std::string  robot_name_;
  double       eef_step_{0.005};
  double       jump_thresh_{20.0};
  double       vel_scale_{0.2};
  double       acc_scale_{0.2};
  double       time_scale_{1.0};

  enum class Stage { NONE, PRE_PICK, PICK, POST_PICK, PRE_PLACE, PLACE, POST_PLACE };
  Stage        stage_{Stage::NONE};
  int          active_id_{-1};
  int          pending_id_{-1};
  std::atomic<bool> busy_{false};

  // Execution threading & cancellation
  std::atomic<bool> request_cancel_{false};
  std::thread       sequence_thread_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  // --- Gripper helpers
  void gripper_set(double width)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = { width };
    gripper_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Gripper -> width=%.3f", width);
  }
  void gripper_open()  { gripper_set(0.08); }
  void gripper_close() { gripper_set(0.0385); }

  // --- Planning & execution (Cartesian)
  bool plan_and_execute_cartesian_(const geometry_msgs::msg::Pose &target, const std::string &label)
  {
    if (request_cancel_) return false;

    // Fresh state + lock orientation to current
    move_group_->setStartStateToCurrentState();
    const auto start_pose = move_group_->getCurrentPose().pose;

    auto corrected_target = target;
    corrected_target.orientation.x = start_pose.orientation.x;
    corrected_target.orientation.y = start_pose.orientation.y;
    corrected_target.orientation.z = start_pose.orientation.z;
    corrected_target.orientation.w = start_pose.orientation.w;

    std::vector<geometry_msgs::msg::Pose> waypoints{ start_pose, corrected_target };


    moveit_msgs::msg::RobotTrajectory traj;
    const double eef_step = eef_step_;
    const double jump_threshold = jump_thresh_;
    const bool avoid_collisions = true;


    double fraction = move_group_->computeCartesianPath(
    waypoints, eef_step, jump_threshold, traj, avoid_collisions);

    if (request_cancel_) {  // NEW: bail if preempt during planning
      RCLCPP_INFO(get_logger(), "Cancelled before time-parameterization for %s", label.c_str());
      return false;
    }

    if (fraction < 0.90) {
      RCLCPP_ERROR(get_logger(), "Cartesian plan %s only %.1f%% complete (step=%.3f m)",
                  label.c_str(), fraction * 100.0, eef_step_);
      return false;
    }
    RCLCPP_INFO(get_logger(), "Cartesian plan %s %.1f%% complete", label.c_str(), fraction*100.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;


    auto current_state = move_group_->getCurrentState(0.5);
    if (!current_state) {
      RCLCPP_WARN(get_logger(), "No current state for time parameterization; executing raw trajectory.");
      plan.trajectory_ = traj;
    } else {
      robot_trajectory::RobotTrajectory rt(current_state->getRobotModel(), move_group_->getName());
      rt.setRobotTrajectoryMsg(*current_state, traj);


      // Time parameterization (from working node; scales are parameters)
      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      bool timed_ok = totg.computeTimeStamps(rt, vel_scale_, acc_scale_);


      if (!timed_ok) {
        RCLCPP_WARN(get_logger(), "Time parameterization failed; executing raw trajectory.");
        plan.trajectory_ = traj;
      } else {
        rt.getRobotTrajectoryMsg(plan.trajectory_);


        if (time_scale_ != 1.0) {
          auto &pts = plan.trajectory_.joint_trajectory.points;
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
    }


    if (request_cancel_) {
      RCLCPP_INFO(get_logger(), "Cancelled before execute for %s", label.c_str());
      return false;
    }


    // Execute once
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

  // --- Go home (cartesian attempt; falls back to joint planning if needed)
  void go_to_home_pose(bool preempt)
  {
    auto begin = [this]() {
      cancel_sequence_thread_if_any_();
      request_cancel_ = false;
      busy_ = true;
      active_id_ = 0;
      stage_ = Stage::NONE;

      sequence_thread_ = std::thread([this]() {
        bool ok = plan_and_execute_cartesian_(home_pose_, "home");
        if (!ok) {
          RCLCPP_WARN(get_logger(), "Cartesian → home failed; trying joint-space planning fallback.");
          move_group_->setPoseTarget(home_pose_);
          auto ec = move_group_->move();
          ok = (ec == moveit::core::MoveItErrorCode::SUCCESS);
        }

        std::lock_guard<std::mutex> lk(mutex_);
        stage_ = Stage::NONE;
        busy_ = false;
        active_id_ = -1;
        request_cancel_ = false;

        // If a pending id exists, start it (via executor timer)
        if (pending_id_ != -1) {
          int next = pending_id_;
          pending_id_ = -1;
          busy_ = true;
          kick_timer_ = this->create_wall_timer(0ms, [this, next]() {
            kick_timer_->cancel();
            if (next == 0) {
              go_to_home_pose(/*preempt=*/false);
            } else {
              start_sequence_async_(next);
            }
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

  // --- Sequence management
  void start_sequence_async_(int id)
  {
    cancel_sequence_thread_if_any_();
    request_cancel_ = false;
    active_id_ = id;
    stage_ = Stage::PRE_PICK;

    sequence_thread_ = std::thread([this, id]() {
      // Look up the pose set
      Sextet steps;
      {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = targets_.find(id);
        if (it == targets_.end()) {
          RCLCPP_ERROR(get_logger(), "No pose targets configured for id=%d. Going home.", id);
          busy_ = false;
          active_id_ = -1;
          request_cancel_ = false;
          go_to_home_pose(/*preempt=*/false);
          return;
        }
        steps = it->second;
      }

      // PRE-PICK
      if (request_cancel_) return;
      if (!plan_and_execute_cartesian_(steps.pre_pick, "pre_pick")) { finish_sequence_(false); return; }

      // If preempt requested BEFORE PICK, honor it
      if (request_cancel_) { finish_sequence_(false); return; }

      // PICK
      stage_ = Stage::PICK;
      if (!plan_and_execute_cartesian_(steps.pick, "pick")) { finish_sequence_(false); return; }
      // Close gripper at pick
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
      // Open gripper at place
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
      RCLCPP_INFO(get_logger(), "Sequence for id=%d complete.", active_id_);
      completed_.insert(active_id_);
    } else {
      RCLCPP_WARN(get_logger(), "Sequence for id=%d ended early.", active_id_);
    }

    stage_ = Stage::NONE;
    busy_  = false;
    request_cancel_ = false;
    active_id_ = -1;

    if (pending_id_ != -1) {
      int next = pending_id_;
      pending_id_ = -1;
      busy_ = true;

      // Post to executor so we're not inside the sequence thread anymore
      kick_timer_ = this->create_wall_timer(0ms, [this, next]() {
        kick_timer_->cancel();
        if (next == 0) {
          go_to_home_pose(/*preempt=*/false);
        } else {
          start_sequence_async_(next); // safe: now on executor thread
        }
      });
    }
  }

  void cancel_sequence_thread_if_any_()
  {
    if (!sequence_thread_.joinable()) return;

    // If we're *in* the sequence thread, don't join (would deadlock)
    if (std::this_thread::get_id() == sequence_thread_.get_id()) {
      request_cancel_ = true;
      try { move_group_->stop(); } catch (...) {}
      return; // let the thread unwind naturally
    }

    request_cancel_ = true;
    try { move_group_->stop(); } catch (...) {}
    sequence_thread_.join();
  }

  // --- Subscribers
  void restartCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      RCLCPP_INFO(get_logger(), "Task restart 'false' — no action.");
      return;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    completed_.clear();
    RCLCPP_WARN(get_logger(), "Task restart received — cleared completed targets.");
  }

  void targetCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const int id_rcv = msg->data;

    const bool valid = (targets_.find(id_rcv) != targets_.end());
    const bool home_request = (id_rcv == 0) || !valid;

    // Same id already in progress?
    if (busy_ && id_rcv == active_id_) {
      RCLCPP_DEBUG(get_logger(), "Already executing id=%d; ignoring duplicate.", id_rcv);
      return;
    }

    // Already completed?
    if (!home_request) {
      std::lock_guard<std::mutex> lk(mutex_);
      if (completed_.count(id_rcv)) {
        RCLCPP_DEBUG(get_logger(), "Target %d already completed; ignoring.", id_rcv);
        return;
      }
    }

    // If busy with something else: preempt only if before PICK stage (or when coming from HOME)
    if (busy_ && id_rcv != active_id_) {
      if (active_id_ == 0 || stage_ == Stage::PRE_PICK) {
        const int next = home_request ? 0 : id_rcv;  // 0 denotes HOME
        RCLCPP_INFO(get_logger(), "Preempting from id=%d (stage=%d) to %s",
                    active_id_, static_cast<int>(stage_), next==0?"HOME":"target");

        // Non-blocking preempt: record the pending id, signal cancel, stop controller
        pending_id_ = next;
        request_cancel_ = true;
        try { move_group_->stop(); } catch (...) {}
        RCLCPP_INFO(get_logger(), "Preempt signalled; current execution will abort shortly.");
        // DO NOT join or start new sequence here; finish_sequence_()/go_to_home_pose() will handoff
      } else {
        RCLCPP_INFO(get_logger(),
          "Ignoring new id=%d; already committed to id=%d (stage=%d).",
          id_rcv, active_id_, static_cast<int>(stage_));
      }
      return;
    }

    // Not busy → start something now
    busy_ = true;
    if (home_request) {
      RCLCPP_WARN(get_logger(), "Home request or invalid target id=%d → going to HOME.", id_rcv);
      go_to_home_pose(/*preempt=*/false);
    } else {
      start_sequence_async_(id_rcv);
    }
  }
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
