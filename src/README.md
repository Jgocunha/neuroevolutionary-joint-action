# Controlling the LBR IIWA in ROS 2 Humble

We assume you have the **lbr\_fri\_ros2\_stack** installed and have launched the IIWA in Gazebo as per the official instructions. For example, the stack provides a Gazebo launch that starts the robot\_state\_publisher, Gazebo, the `ros2_control_node`, and loads the `joint_trajectory_controller` for the IIWA. You can launch the IIWA-14 (R820) simulation with a command like:

```bash
ros2 launch lbr_bringup gazebo.launch.py model:=iiwa14
```

This will bring up RViz and Gazebo with the IIWA model and ROS 2 control ready. In this setup, the **joint\_trajectory\_controller** action server is already active (MoveIt logs confirm a FollowJointTrajectory controller is available). We will now build C++ examples that connect to this controller.

## Joint-Space Position Control (FollowJointTrajectory Action)

To send joint-space goals, we write a C++ node that is an action client for the `control_msgs::action::FollowJointTrajectory` action. The default IIWA controller advertises this action (e.g. `joint_trajectory_controller/follow_joint_trajectory`). The steps are:

1. **Create a ROS 2 package.** In your workspace (e.g. `ros2_ws/src`), run:

   ```bash
   ros2 pkg create iiwa_joint_control --build-type ament_cmake \
     --dependencies rclcpp rclcpp_action control_msgs trajectory_msgs \
   ```

   This adds dependencies for rclcpp, rclcpp\_action, and the message packages (ensure `control_msgs` and `trajectory_msgs` are listed in `package.xml` and `find_package` in `CMakeLists.txt`).

2. **Write the action-client code.** In `src/` of the package, create (for example) `joint_control.cpp`. A minimal example:

   ```cpp
  #include <memory>
  #include <vector>
  #include <string>
  #include <rclcpp/rclcpp.hpp>
  #include <rclcpp_action/rclcpp_action.hpp>
  #include <control_msgs/action/follow_joint_trajectory.hpp>
  #include <trajectory_msgs/msg/joint_trajectory_point.hpp>

  class JointCommander : public rclcpp::Node {
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
   ```

   This node creates a `FollowJointTrajectory` action client and sends a one-point trajectory command. Adjust the joint names and positions as needed for your task. The above uses the standard IIWA joint names from the LBR stack.

3. **Build and run the joint control node.** In `CMakeLists.txt`, add `rclcpp`, `rclcpp_action`, `control_msgs`, and `trajectory_msgs` to `find_package()` and link your executable against them. For example:

   ```cmake
   # find dependencies
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

   ```bash
   colcon build --packages-select iiwa_joint_control
   source install/setup.bash
   ```

4. **Test joint commands.** First make sure the simulation is running (Gazebo + controllers). In another terminal, launch the node:

   ```bash
   ros2 run iiwa_joint_control joint_control
   ```

   You should see log output, and the simulated IIWA should move to the commanded joint configuration over about 2 seconds. You can verify the action by monitoring `/joint_trajectory_controller/joint_trajectory` or observing RViz/Gazebo. (Use `ros2 action list` to see that `/joint_trajectory_controller/follow_joint_trajectory` is available.)

## Testing and Tips

* **Controllers:** Verify the controllers are running with `ros2 control list_controllers`. The joint trajectory controller should be listed as `active`. If not, load it (the Gazebo bringup usually does this automatically).

* **Namespaces:** If you launch multiple robots or use non-default names, the joint controller’s namespace may differ (e.g. `robot_name/joint_trajectory_controller`). Adjust the action/client names accordingly.

* **MoveIt Setup:** The above MoveIt example assumes you have a working MoveGroup node. If you don’t have a full MoveIt config, you could alternatively solve the IK manually (e.g. using `moveit::core::RobotState` and `KinematicsQueryOptions`) and send a joint goal via the action client. But using MoveIt2 is recommended for convenience.

* **Timing:** In action-client code, we gave a `time_from_start` of 2 seconds. Adjust timing and trajectory length as needed. The FollowJointTrajectory action will execute at the controller’s rate until reaching the point.

* **Debugging:** Use `ros2 topic echo /joint_states` or RViz to see if joints move. Also `ros2 action send_goal` (CLI) can be used to test the controller action server interactively.

By following the above steps, you should be able to command the simulated IIWA in joint space via a simple C++ action client, and in Cartesian space via a MoveIt2 planner. These examples can be integrated into the existing lbr\_fri\_ros2\_stack or built as standalone nodes in your workspace.

**Sources:** The LBR-Stack’s documentation and examples describe using ROS 2 control with a joint trajectory controller, and MoveIt2’s C++ interface provides the move group API for pose goals. The MoveIt tutorial also highlights adding `moveit_ros_planning_interface` to the CMake/`package.xml` when using MoveGroupInterface.

## Cartesian-Space Position Control (MoveIt2)

[lbr-stack docs](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_moveit_cpp/doc/lbr_moveit_cpp.html)

1. Run the robot driver:
```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka.launch.py \
    moveit:=true \
    model:=iiwa14 \
    use_sim_time:=true
```

2. Run MoveIt:
```bash
ros2 launch lbr_bringup move_group.launch.py \
    mode:=gazebo \
    model:=iiwa14 \
    rviz:=true \
    use_sim_time:=true
```

3. Run the ```moveit2_control_demo``` node:
```bash
ros2 launch kuka_lbr_iiwa14_marlab moveit2_control_demo.launch.py \
    mode:=gazebo \
    model:=iiwa14 \
    use_sim_time:=true
```

### Current state
```bash
[moveit2_control_demo-1] [INFO] [1750256988.021033037] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[moveit2_control_demo-1] [WARN] [1750256989.122030714] [moveit_ros.current_state_monitor]: No state update received within 100ms of system clock. Have been waiting for 0.988000s, timeout is 1.000000s
[moveit2_control_demo-1] [INFO] [1750256989.122059635] [moveit_ros.current_state_monitor]: Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds. Requested time 15.417000, but latest received state has time 0.000000.
[moveit2_control_demo-1] Check clock synchronization if your are running ROS across multiple machines!
[moveit2_control_demo-1] [ERROR] [1750256989.122074295] [move_group_interface]: Failed to fetch current robot state
[moveit2_control_demo-1] [INFO] [1750256989.122085025] [moveit2_control_demo]: Current end-effector pose: x=0.000, y=0.000, z=0.000
[moveit2_control_demo-1] [INFO] [1750256989.122112775] [moveit2_control_demo]: Planning to target pose: x=0.400, y=0.000, z=0.900

```

Still, robot moves using moveit2.