#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class JointCommander : public rclcpp::Node 
{
public:
    using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
    JointCommander()
    : Node("iiwa_joint_control", "/lbr")
    {
    // Create the action client for FollowJointTrajectory
    action_client_ = rclcpp_action::create_client<TrajectoryAction>(
        this, "/lbr/joint_trajectory_controller/follow_joint_trajectory");
    }

    void send_goal()
    {
    // Wait until the action server is available
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Action server not available");
        return;
    }

    // Create a goal message
    auto goal_msg = TrajectoryAction::Goal();
    // Define joint names (should match the robot description order)
    goal_msg.trajectory.joint_names = {
        "lbr_A1", "lbr_A2", "lbr_A3", 
        "lbr_A4", "lbr_A5", "lbr_A6", "lbr_A7"
    };
    // Create one trajectory point (for example, a slight move)
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, -0.3, 0.0, -1.2, 0.0, 1.5, 0.0};
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    goal_msg.trajectory.points.push_back(point);

    // Send the goal
    RCLCPP_INFO(get_logger(), "Sending joint trajectory goal...");
    auto send_opts = rclcpp_action::Client<TrajectoryAction>::SendGoalOptions();
    send_opts.result_callback = 
        [this](const rclcpp_action::ClientGoalHandle<TrajectoryAction>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed");
        }
        };
    action_client_->async_send_goal(goal_msg, send_opts);
    }

private:
    rclcpp_action::Client<TrajectoryAction>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointCommander>();
    // Give ROS time to connect to server
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    node->send_goal();
    // Keep node alive to process callbacks
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}