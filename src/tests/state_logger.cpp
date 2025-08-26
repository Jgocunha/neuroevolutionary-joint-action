#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

using namespace std::chrono_literals;

// https://github.com/moveit/moveit2_tutorials/issues/958
// Prefer a MultiThreadedExecutor + separate callback group (or the separate MGI executor) 
// any time you use MoveGroupInterface in a node that also has timers/services.
// If you only need a snapshot occasionally, consider a slower timer (e.g., 1s) or call getCurrentState(0.1) to avoid long blocking.
// You can also use:
// move_group_->getCurrentStateMonitor()->waitForCurrentState(rclcpp::Duration::from_seconds(0.5));



class StatePrinter : public rclcpp::Node {
public:
  StatePrinter(const rclcpp::NodeOptions & options)
  : Node("state_printer", options)
  {
    // put the timer in its own (reentrant) callback group
    timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

  void init(const rclcpp::Node::SharedPtr & self) {
    std::string robot_name = get_parameter("robot_name").as_string();

    auto opts = moveit::planning_interface::MoveGroupInterface::Options(
        "arm", "robot_description", robot_name);

    // make sure we read joint states from the right place (harmless even if already remapped)
    //opts.node_options.append_parameter_override("joint_state_topic", "/lbr/joint_states");

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(self, opts);

    // attach the timer to the separate group
    timer_ = rclcpp::create_timer(
        this, this->get_clock(), 500ms,
        std::bind(&StatePrinter::print_state, this),
        timer_group_);
  }

private:
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;

  void print_state()
  { 
    // Current joint values
    auto state = move_group_->getCurrentState(1.0);
    if (state) {
      std::vector<double> q;
      state->copyJointGroupPositions(move_group_->getName(), q);
      std::ostringstream oss;
      oss << "Joints: ";
      for (size_t i = 0; i < q.size(); ++i) {
        oss << "q" << i << "=" << q[i] << " ";
      }
      RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
    } else {
      RCLCPP_WARN(get_logger(), "No joint state received.");
    }

    // Current end-effector pose
    auto pose_stamped = move_group_->getCurrentPose();
    auto &p = pose_stamped.pose;
    RCLCPP_INFO(get_logger(),
      "Pose: position(%.3f, %.3f, %.3f) orientation(%.3f, %.3f, %.3f, %.3f)",
      p.position.x, p.position.y, p.position.z,
      p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<StatePrinter>(options);
  node->init(node);

  // SPIN WITH MULTITHREADED EXECUTOR
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
