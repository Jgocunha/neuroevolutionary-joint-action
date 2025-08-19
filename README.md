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

## Running hardware tests

### 1. Launch Simulation Environment

```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka_env.launch.py \
 moveit:=true model:=iiwa14 use_sim_time:=true
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

---

## Running the robot task

### 1. Launch Simulation Environment

```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka_env.launch.py \
 moveit:=true model:=iiwa14 use_sim_time:=true
```

### 2. Launch OnRobot Driver
```bash
ros2 launch kuka_lbr_iiwa14_marlab low_level_control_node.launch.py \
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

## References

* [lbr\_fri\_ros2\_stack Docs](https://lbr-stack.readthedocs.io/)

---
