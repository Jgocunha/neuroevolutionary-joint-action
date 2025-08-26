# KUKA LBR iiwa 14 Control with ROS 2

This package demonstrates how to control the **KUKA LBR iiwa 14** robot using **joint-space control** or **cartesian-space control via MoveIt 2**. It has been tested on **Ubuntu 22.04** with **ROS 2 Humble** and assumes you have installed the [`lbr_fri_ros2_stack`](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/lbr_fri_ros2_stack.html) and have the IIWA simulated in Gazebo.

---

## Prerequisites

- ROS 2 Humble
- [`lbr_fri_ros2_stack`](https://lbr-stack.readthedocs.io/)
- [`marlab_env_description`](https://github.com/Jgocunha/marlab_env_description) (for Gazebo worlds)
- [`onrobot_ros2_driver`](https://github.com/Jgocunha/OnRobot_ROS2_Driver)
- [`onrobot_rg_gazebo`](https://github.com/Jgocunha/onrobot_rg_gazebo)
- Gazebo with ROS 2 integration
- MoveIt 2

---

## Running the task

### 1. Launch Simulation Environment

```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka_move_group_env.launch.py  \
 moveit:=true model:=iiwa14 use_sim_time:=true rviz:=true mode:=gazebo
```

### 2. Launch OnRobot Driver

```bash
ros2 launch onrobot_driver onrobot_control.launch.py \
  onrobot_type:=rg2 connection_type:=tcp use_fake_hardware:=true
```

Tip: You might have to run this command twice if the gripper doesn't fully load in Rviz.

### 3. Launch joint_control test node

```bash
ros2 launch kuka_lbr_iiwa14_marlab joint_control.launch.py \
 mode:=gazebo model:=iiwa14
```

### 4. Launch onrobot_rg2_control test node

```bash
ros2 launch kuka_lbr_iiwa14_marlab onrobot_rg2_control.launch.py
```

### 5. Launch cartesian_control test node

```bash
ros2 launch kuka_lbr_iiwa14_marlab cartesian_path_planning.launch.py \
 mode:=gazebo model:=iiwa14
```

---

## Running the robot task

### 1. Launch Simulation Environment

```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka_move_group_env.launch.py  \
 moveit:=true model:=iiwa14 use_sim_time:=true rviz:=true mode:=gazebo
```

### 2. Launch OnRobot Driver

```bash
ros2 launch onrobot_driver onrobot_control.launch.py \
  onrobot_type:=rg2 connection_type:=tcp use_fake_hardware:=true
```

Tip: You might have to run this command twice if the gripper doesn't fully load in Rviz.

### 3. Run the low-level control node

```bash
ros2 launch kuka_lbr_iiwa14_marlab low_level_control_node.launch.py  \
 mode:=gazebo model:=iiwa14  mode:=gazebo model:=iiwa14
```

### 4. Run the high-level control node (dnf-architecture)

```bash
ros2 launch kuka_lbr_iiwa14_marlab high_level_control_node.launch.py
```

---

## Hardware setup

Take a look at the `hardware.md` file in this repo.

---

## References

- [lbr_fri_ros2_stack Docs](https://lbr-stack.readthedocs.io/)

---

---

# TESTING.

# Simulation

## onrobot

1. ros2 launch onrobot_driver onrobot_control.launch.py \
   onrobot_type:=rg2 connection_type:=tcp use_fake_hardware:=true
2. ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
3. ros2 launch kuka_lbr_iiwa14_marlab onrobot_rg2_control.launch.py

## kuka

1. ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka_move_group_env.launch.py \
   moveit:=true \
   mode:=gazebo \
   model:=iiwa14 \
   rviz:=true

2. ros2 launch kuka_lbr_iiwa14_marlab state_logger.launch.py mode:=gazebo model:=iiwa14

2. ros2 launch kuka_lbr_iiwa14_marlab joint_control.launch.py mode:=gazebo model:=iiwa14

3. ros2 launch kuka_lbr_iiwa14_marlab cartesian_path_planning.launch.py mode:=gazebo model:=iiwa14 

# Hardware

## onrobot

5. ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=rg2 connection_type:=tcp ip_address:=172.31.1.4
6. ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
7. ros2 launch kuka_lbr_iiwa14_marlab onrobot_rg2_control.launch.py

## kuka

1. **Launch LBRServer on SmartPad**

   - FRI send period: `10 ms`
   - FRI control mode: `POSITION_CONTROL`
   - Client command mode: `POSITION`
   - IP: `192.168.11.2` (your PC’s IP)

2. ros2 launch kuka_lbr_iiwa14_marlab marlab_hardware.launch.py \
   moveit:=true \
   mode:=hardware \
   model:=iiwa14 \
   rviz:=true

3. Listening to robot state
   ros2 launch kuka_lbr_iiwa14_marlab state_logger.launch.py mode:=hardware model:=iiwa14

4. ros2 launch kuka_lbr_iiwa14_marlab joint_control.launch.py mode:=hardware model:=iiwa14

5. ros2 launch kuka_lbr_iiwa14_marlab cartesian_path_planning.launch.py mode:=hardware model:=iiwa14 

---

# Task details

- table centre point in relation to robot with onrobotrg2 
Joints: q0=-1.87881 q1=-1.86354 q2=1.07392 q3=-0.871955 q4=0.271321 q5=0.023817 q6=0.134271 
Pose: position(0.662, -0.005, 0.951) orientation(0.847, -0.120, 0.509, 0.098)


# Roadmap

- define table and object positions
- create test/object_pose_path_planning with pick, pre-pick, post-pick, (place)