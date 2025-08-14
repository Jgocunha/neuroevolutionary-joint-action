#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class OnRobotControl : public rclcpp::Node
{
public:
  explicit OnRobotControl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("onrobot_control", options)
  {
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/onrobot/finger_width_controller/commands", 10);

    const int open_close_iterations = 5;
    const int wait_time = 1000;  // milliseconds

    for (int i = 0; i < open_close_iterations; ++i)
    {
      gripper_open();
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
      gripper_close();
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  const double GRIPPER_OPEN  = 0.10;
  const double GRIPPER_CLOSE = 0.05;

  void gripper_set(double width)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = { width };
    gripper_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Gripper cmd: width=%.3f", width);
  }
  void gripper_open()  { gripper_set(GRIPPER_OPEN);  }
  void gripper_close() { gripper_set(GRIPPER_CLOSE); }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<OnRobotControl>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
