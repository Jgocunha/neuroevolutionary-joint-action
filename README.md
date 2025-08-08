# KUKA LBR iiwa 14 Control with ROS 2

This package demonstrates how to control the **KUKA LBR iiwa 14** robot using **joint-space control** or **cartesian-space control via MoveIt 2**. It has been tested on **Ubuntu 22.04** with **ROS 2 Humble** and assumes you have installed the [`lbr_fri_ros2_stack`](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/lbr_fri_ros2_stack.html) and have the IIWA simulated in Gazebo.

---

## Prerequisites

* ROS 2 Humble
* [`lbr_fri_ros2_stack`](https://lbr-stack.readthedocs.io/)
* [`marlab_env_description`](https://github.com/Jgocunha/marlab_env_description) (for Gazebo worlds)
* Gazebo with ROS 2 integration
* MoveIt 2

---

## Joint-Space Position Control (FollowJointTrajectory Action)

### 1. Create a ROS 2 Package

```bash
cd ~/ros2_ws/src
ros2 pkg create iiwa_joint_control --build-type ament_cmake \
  --dependencies rclcpp rclcpp_action control_msgs trajectory_msgs
```

Ensure the following dependencies are listed in both `package.xml` and `CMakeLists.txt`.

### 2. Write the Action Client Node

Save the following to `src/joint_control.cpp`:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class JointCommander : public rclcpp::Node {
public:
  using TrajectoryAction = control_msgs::action::FollowJointTrajectory;

  JointCommander() : Node("iiwa_joint_control", "/lbr") {
    action_client_ = rclcpp_action::create_client<TrajectoryAction>(
        this, "/lbr/joint_trajectory_controller/follow_joint_trajectory");
  }

  void send_goal() {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = TrajectoryAction::Goal();
    goal_msg.trajectory.joint_names = {
        "lbr_A1", "lbr_A2", "lbr_A3", "lbr_A4", "lbr_A5", "lbr_A6", "lbr_A7"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, -0.3, 0.0, -1.2, 0.0, 1.5, 0.0};
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    goal_msg.trajectory.points.push_back(point);

    RCLCPP_INFO(get_logger(), "Sending joint trajectory goal...");

    auto send_opts = rclcpp_action::Client<TrajectoryAction>::SendGoalOptions();
    send_opts.result_callback = [this](const auto &result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Trajectory executed successfully");
      } else {
        RCLCPP_ERROR(get_logger(), "Trajectory execution failed");
      }
    };

    action_client_->async_send_goal(goal_msg, send_opts);
  }

private:
  rclcpp_action::Client<TrajectoryAction>::SharedPtr action_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointCommander>();
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  node->send_goal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 3. Update `CMakeLists.txt`

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(joint_control src/joint_control.cpp)
ament_target_dependencies(joint_control 
  rclcpp 
  rclcpp_action 
  control_msgs 
  trajectory_msgs)

install(TARGETS joint_control DESTINATION lib/${PROJECT_NAME})
```

### 4. Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select iiwa_joint_control
source install/setup.bash
```

### 5. Launch and Test

Make sure Gazebo and controllers are running:

```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka.launch.py \
  moveit:=true model:=iiwa14 use_sim_time:=true
```

In another terminal:

```bash
ros2 run iiwa_joint_control joint_control
```

You should observe the robot moving to the desired joint configuration in Gazebo.

---

## Cartesian-Space Control with MoveIt 2

Follow the [official MoveIt C++ interface docs](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_moveit_cpp/doc/lbr_moveit_cpp.html).

### 1. Launch Simulation

```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka.launch.py \
  moveit:=true model:=iiwa14 use_sim_time:=true
```

### 2. Launch MoveIt

```bash
ros2 launch lbr_bringup move_group.launch.py \
  mode:=gazebo model:=iiwa14 rviz:=true use_sim_time:=true
```

### 3. Run the Cartesian Control Demo

```bash
ros2 launch kuka_lbr_iiwa14_marlab moveit2_control_demo.launch.py \
  mode:=gazebo model:=iiwa14 use_sim_time:=true
```

Example C++ node:

```cpp
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node_ptr = rclcpp::Node::make_shared("moveit2_control_demo");
  node_ptr->declare_parameter("robot_name", "lbr");
  auto robot_name = node_ptr->get_parameter("robot_name").as_string();

  auto move_group_interface = moveit::planning_interface::MoveGroupInterface(
      node_ptr, moveit::planning_interface::MoveGroupInterface::Options(
                    "arm", "robot_description", robot_name));
  move_group_interface.setStartStateToCurrentState();

  auto current_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(node_ptr->get_logger(), "Current pose: x=%.3f y=%.3f z=%.3f",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.9;

  move_group_interface.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto success = move_group_interface.plan(plan);

  if (success == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(node_ptr->get_logger(), "Failed to plan to target pose");
  }

  rclcpp::shutdown();
  return 0;
}
```
---

## Running the entire setup

### 1. Launch Simulation

```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka.launch.py \
  moveit:=true model:=iiwa14 use_sim_time:=true
```

### 2. Launch OnRobot Driver
```bash
ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=rg2 connection_type:=tcp use_fake_hardware:=true
```

### 3. Launch MoveIt

```bash
ros2 launch lbr_bringup move_group.launch.py \
  mode:=gazebo model:=iiwa14 rviz:=true use_sim_time:=true
```

### 4. Run the Cartesian Control Demo

```bash
ros2 launch kuka_lbr_iiwa14_marlab moveit2_control_demo.launch.py \
  mode:=gazebo model:=iiwa14 use_sim_time:=true
```

### 5. Control the Gripper

```bash
ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
```

### 6. Spawn the Objects

```bash
ros2 launch marlab_env_description spawn_objects.launch.py 
```

---
## Hardware setup

### KUKA LBR iiwa 14

[lbr\_fri\_ros2\_stack docs hardware setup](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html)

1. You will need to be connected via ethernet to the robot's controller KONI port;
2. Set your PC's IP address to 192.168.2, netmask 255.255.255.0 and gateway 192.168.11.2;
3. KUKA left arm IP address on the x66 port is 172.31.1.148, the right arm is 172.31.1.147;
4. The KONI port IP address was set via the Sunrise Workbench application to 192.168.1.100;
5. When following the `Install applications to the robot` instructions on the lbr-stack docs, when cloning the `fri` repository checkout to branch `ros2-fri1.15`;
6. When following the `Install applications to the robot` instructions on the lbr-stack docs, at the end you will have to edit the `LBRServer.java` file manually to set the IP addresses of your machine accordingly (192.168.1.2 if using the KONI port).

Finally take a look at the `hardware.md` file in this repo.

### OnRobot (RG6)
1. You have to be connected to the MARLab wi-fi;
2. OnRobot's controller IP address of the left arm is 172.31.1.4.

---

## Dependencies

Make sure the following are declared in your `package.xml`:

```xml
<depend>rclcpp</depend>
<depend>rclcpp_action</depend>
<depend>control_msgs</depend>
<depend>trajectory_msgs</depend>
<depend>geometry_msgs</depend>
<depend>moveit_ros_planning_interface</depend>
```

---

## References

* [lbr\_fri\_ros2\_stack Docs](https://lbr-stack.readthedocs.io/)

---
