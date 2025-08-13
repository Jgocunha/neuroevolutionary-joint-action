// mock_pick_place_jointspace.cpp  (ROS 2 Humble, C++17)

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>
#include <chrono>
#include <atomic>
#include <functional>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int32.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class MockPickPlaceJointspace : public rclcpp::Node
{
public:
  using TrajAction  = control_msgs::action::FollowJointTrajectory;
  using GoalHandle  = rclcpp_action::ClientGoalHandle<TrajAction>;
  using JointArray  = std::array<double, 7>;

  struct Sextet {
    JointArray pre_grasp;
    JointArray grasp;
    JointArray post_grasp;
    JointArray pre_place;
    JointArray place;
    JointArray post_place;
  };

  explicit MockPickPlaceJointspace(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mock_pick_place_jointspace", options)
  {
    robot_name_ = get_parameter("robot_name").as_string();
    move_time_s_ = this->declare_parameter<double>("move_time_s", 5.0);

    action_name_ = "/" + robot_name_ + "/joint_trajectory_controller/follow_joint_trajectory";
    action_client_ = rclcpp_action::create_client<TrajAction>(this, action_name_);

    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/onrobot/finger_width_controller/commands", 10);

    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "target_object", 10,
      std::bind(&MockPickPlaceJointspace::targetCallback, this, std::placeholders::_1));

    restart_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "task_restart", 10,
      std::bind(&MockPickPlaceJointspace::restartCallback, this, std::placeholders::_1));

    targets_ = {
      {1, { { -1.74533, 1.48353, 1.83260, -1.65806, -1.57080, 0.0, 1.57080 },
            { -1.48353, 1.48353, 1.83260, -1.30899, -1.57080, 0.0, 1.57080 },
            { -1.74533, 1.48353, 1.83260, -1.65806, -1.57080, 0.0, 1.57080 },
            { -1.308997, 1.570796, 2.356194, -1.396263, -1.570796, 0.0, 1.570796 },
            { -1.570796, 1.570796, 2.356194, -1.396263, -1.570796, 0.0, 1.570796 },
            { -1.308997, 1.570796, 2.356194, -1.396263, -1.570796, 0.0, 1.570796 } }},
      {2, { { -1.047198, 1.954769, 1.745329, -0.959931, -1.570796, -0.610865, 1.308997 },
            { -1.134464, 1.954769, 1.745329, -0.959931, -1.570796, -0.610865, 1.308997 },
            { -1.047198, 1.954769, 1.745329, -0.959931, -1.570796, -0.610865, 1.308997 },
            { -1.308997, 1.570796, 2.356194, -1.396263, -1.570796, 0.0, 1.570796 },
            { -1.570796, 1.570796, 2.356194, -1.396263, -1.570796, 0.0, 1.570796 },
            { -1.308997, 1.570796, 2.356194, -1.396263, -1.570796, 0.0, 1.570796 } }},
      {3, { { -0.523599, 1.832596, 0.698132, -0.261799, -1.483530, -0.610865, 2.094395 },
            { -0.698132, 1.832596, 0.698132, -0.261799, -1.483530, -0.610865, 2.094395 },
            { -0.523599, 1.832596, 0.698132, -0.261799, -1.483530, -0.610865, 2.094395 },
            { -1.308997, 1.570796, 2.356194, -1.396263, -1.570796, 0.0, 1.570796 },
            { -1.570796, 1.570796, 2.356194, -1.396263, -1.570796, 0.0, 1.570796 },
            { -1.308997, 1.570796, 2.356194, -1.396263, -1.570796, 0.0, 1.570796 } }},
    };

    RCLCPP_INFO(get_logger(),
      "Ready. Waiting on 'target_object'. Action server: %s", action_name_.c_str());

    startup_timer_ = this->create_wall_timer(300ms, [this]() {
      startup_timer_->cancel();
      gripper_open();
      go_to_start_pose_immediate();
    });
  }

private:
  // ROS handles
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr           sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr            restart_sub_;
  rclcpp_action::Client<TrajAction>::SharedPtr                    action_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  gripper_pub_;

  // Data/state
  std::map<int, Sextet>  targets_;
  std::string            robot_name_;
  std::string            action_name_;
  std::mutex             mutex_;
  std::set<int>          completed_;

  double                 move_time_s_{2.0};
  std::atomic<bool>      busy_{false};
  int                    active_id_{-1};

  enum class Stage { NONE, PRE_GRASP, GRASP, POST_GRASP, PRE_PLACE, PLACE, POST_PLACE };
  Stage                  stage_{Stage::NONE};

  Sextet                 active_steps_{};
  typename GoalHandle::SharedPtr active_goal_;

  rclcpp::TimerBase::SharedPtr delay_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;

  JointArray             start_pose_{-1.5708, 1.5708, 1.5708, -1.5708, -1.5708, 0.0, 1.5708};

  // Gripper widths
  const double GRIPPER_OPEN  = 0.50;
  const double GRIPPER_CLOSE = 0.05;

  static constexpr int START_ID = 0;

  static const std::array<const char*, 7> & jointNames()
  {
    static const std::array<const char*, 7> names{
      "lbr_A1","lbr_A2","lbr_A3","lbr_A4","lbr_A5","lbr_A6","lbr_A7"
    };
    return names;
  }

  // --- gripper helpers
  void gripper_set(double width)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = { width };
    gripper_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Gripper cmd: width=%.3f", width);
  }
  void gripper_open()  { gripper_set(GRIPPER_OPEN);  }
  void gripper_close() { gripper_set(GRIPPER_CLOSE); }

  // Build a single-point trajectory goal
  TrajAction::Goal make_goal_(const JointArray & joints, double duration_s)
  {
    TrajAction::Goal goal;
    goal.trajectory.header.stamp = this->now(); // start ASAP
    goal.trajectory.joint_names.assign(jointNames().begin(), jointNames().end());
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(joints.begin(), joints.end());
    pt.time_from_start = rclcpp::Duration::from_seconds(duration_s);
    goal.trajectory.points.clear();
    goal.trajectory.points.push_back(pt);
    return goal;
  }

  // Send async goal; store goal handle; on result, call on_done(success)
  void send_joint_goal_async_(
      const JointArray & joints,
      const std::string & label,
      std::optional<int> result_filter_id, // std::nullopt = never filter
      std::function<void(bool)> on_done)
  {
    int shown_id = result_filter_id.value_or(-999);
    RCLCPP_INFO(get_logger(),
      "Sending %s goal: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] in %.2fs (id=%d)",
      label.c_str(),
      joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6],
      move_time_s_, shown_id);

    auto goal = make_goal_(joints, move_time_s_);

    typename rclcpp_action::Client<TrajAction>::SendGoalOptions opts;
    opts.goal_response_callback = [this, label](typename GoalHandle::SharedPtr handle){
      if (!handle) {
        RCLCPP_ERROR(this->get_logger(), "%s goal rejected.", label.c_str());
      } else {
        std::lock_guard<std::mutex> lk(this->mutex_);
        this->active_goal_ = handle; // last handle wins
        RCLCPP_INFO(this->get_logger(), "%s goal accepted.", label.c_str());
      }
    };
    opts.result_callback =
      [this, label, on_done, result_filter_id](const typename GoalHandle::WrappedResult & wrapped)
    {
      if (result_filter_id.has_value() && *result_filter_id != this->active_id_) {
        RCLCPP_INFO(this->get_logger(),
          "Ignoring stale result for %s (old id=%d, current id=%d).",
          label.c_str(), *result_filter_id, this->active_id_);
        return;
      }
      const bool ok = (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED);
      if (ok) {
        RCLCPP_INFO(this->get_logger(), "%s executed successfully.", label.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s execution finished with code=%d.",
                     label.c_str(), static_cast<int>(wrapped.code));
      }
      on_done(ok);
    };

    action_client_->async_send_goal(goal, opts);
  }

  // one-shot delay (used between steps)
  void arm_delay_(std::chrono::milliseconds d, std::function<void()> fn)
  {
    if (delay_timer_) delay_timer_->cancel();
    delay_timer_ = this->create_wall_timer(d, [this, fn]() {
      if (delay_timer_) delay_timer_->cancel(); // one-shot
      fn();
    });
  }

  // Clear queued timers so nothing from the previous plan fires
  void clear_pending_()
  {
    if (delay_timer_) delay_timer_->cancel();
  }

  // Immediate start-pose (no cancel, just override with a new goal)
  void go_to_start_pose_immediate()
  {
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "FollowJointTrajectory action server not available.");
      return;
    }
    clear_pending_();

    busy_     = true;
    active_id_ = START_ID;
    stage_     = Stage::NONE;

    send_joint_goal_async_(start_pose_, "start-pose", std::nullopt,
      [this](bool ok){
        if (!ok) {
          RCLCPP_WARN(this->get_logger(), "Start pose move failed.");
        } else {
          RCLCPP_INFO(this->get_logger(), "At start pose.");
        }
        std::lock_guard<std::mutex> lk(mutex_);
        stage_ = Stage::NONE;
        active_goal_.reset();
        busy_ = false;
      });
  }

  // Start a new sequence (immediate override)
  void start_sequence_immediate_(int id)
  {
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "FollowJointTrajectory action server not available.");
      return;
    }
    clear_pending_();

    active_id_    = id;
    stage_        = Stage::PRE_GRASP;
    active_steps_ = targets_.at(id);
    busy_         = true;

    RCLCPP_INFO(get_logger(), "Pick & place (joint-space) for id=%d", id);

    // PRE-GRASP
    send_joint_goal_async_(active_steps_.pre_grasp, "pre-grasp", id,
      [this, id](bool ok){
        if (!ok) { finish_sequence_(false); return; }

        stage_ = Stage::GRASP;
        arm_delay_(200ms, [this, id](){
          // GRASP
          send_joint_goal_async_(active_steps_.grasp, "grasp", id,
            [this, id](bool ok2){
              if (!ok2) { finish_sequence_(false); return; }

              gripper_close();
              arm_delay_(150ms, [this, id](){

                stage_ = Stage::POST_GRASP;
                arm_delay_(200ms, [this, id](){
                  // POST-GRASP
                  send_joint_goal_async_(active_steps_.post_grasp, "post-grasp", id,
                    [this, id](bool ok3){
                      if (!ok3) { finish_sequence_(false); return; }

                      stage_ = Stage::PRE_PLACE;
                      arm_delay_(200ms, [this, id](){
                        // PRE-PLACE
                        send_joint_goal_async_(active_steps_.pre_place, "pre-place", id,
                          [this, id](bool ok4){
                            if (!ok4) { finish_sequence_(false); return; }

                            stage_ = Stage::PLACE;
                            arm_delay_(200ms, [this, id](){
                              // PLACE
                              send_joint_goal_async_(active_steps_.place, "place", id,
                                [this, id](bool ok5){
                                  if (!ok5) { finish_sequence_(false); return; }

                                  gripper_open();
                                  arm_delay_(150ms, [this, id](){

                                    stage_ = Stage::POST_PLACE;
                                    arm_delay_(200ms, [this, id](){
                                      // POST-PLACE
                                      send_joint_goal_async_(active_steps_.post_place, "post-place", id,
                                        [this](bool ok6){ finish_sequence_(ok6); });
                                    });

                                  }); // after open
                                });
                            }); // place delay
                          });
                      }); // pre-place delay
                    });
                }); // post-grasp delay

              }); // after close
            });
        }); // grasp delay
      });
  }

  // Finish sequence bookkeeping
  void finish_sequence_(bool success)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Sequence for id=%d complete.", active_id_);
      completed_.insert(active_id_);
    } else {
      RCLCPP_WARN(this->get_logger(), "Sequence for id=%d ended early.", active_id_);
    }
    stage_ = Stage::NONE;
    active_goal_.reset();
    busy_ = false;
  }

  // Hard-preempt policy: NEVER wait for cancel — just override with the new goal.
  void targetCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    const int id = msg->data;
    const bool invalid = (id == 0) || !targets_.count(id);

    if (invalid) {
      RCLCPP_WARN(get_logger(), "Invalid target id: %d → going to START pose (immediate).", id);
      go_to_start_pose_immediate();
      return;
    }

    if (busy_ && id == active_id_) {
      RCLCPP_DEBUG(get_logger(), "Already executing id=%d; ignoring duplicate.", id);
      return;
    }

    RCLCPP_INFO(get_logger(), "Immediate preempt → id=%d", id);
    start_sequence_immediate_(id);
  }

  void restartCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      std::lock_guard<std::mutex> lk(mutex_);
      completed_.clear();
      RCLCPP_WARN(this->get_logger(), "Task restart received — completed targets cleared.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Task restart received with 'false' — no action taken.");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MockPickPlaceJointspace>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
