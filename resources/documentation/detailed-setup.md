# KUKA LBR and OnRobot Hardware & MoveIt Setup

This guide describes how to connect, configure, and operate the **KUKA LBR iiwa 14 R820** robot with **OnRobot RG2** grippers and **MoveIt 2** on **ROS 2 Humble**.  

It includes both **hardware** and **simulation** instructions used in the **neat-dnfs collaborative packaging task** experiment.

**Summary:**  
> - Wired control for robot (Ethernet via KONI port)  
> - Wi-Fi control for gripper (Marlab network)  
> - MoveIt 2 planning, Gazebo simulation, and PTU setup  

---

## Dependencies

Make sure the following packages are installed and sourced before continuing:

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [lbr_fri_ros2_stack](https://lbr-stack.readthedocs.io/)
- [marlab_env_description](https://github.com/Jgocunha/marlab_env_description)
- [onrobot_ros2_driver](https://github.com/Jgocunha/OnRobot_ROS2_Driver)
- [onrobot_rg_gazebo](https://github.com/Jgocunha/onrobot_rg_gazebo)
- [MoveIt 2](https://moveit.picknik.ai/main/doc/getting_started/getting_started.html)
- **Gazebo** with ROS 2 integration

> ⚠️ Some YAML and URDF changes to `lbr_fri_ros2_stack` are temporarily required. See [below](#-required-edits-to-lbr_fri_ros2_stack).
---

## Required Edits to `lbr_fri_ros2_stack`

> ⚠️ These are temporary patches — avoid committing them upstream.

1. **Increase controller update rate:**
   - `lbr_description/ros2_control/lbr_controllers.yaml`
     ```yaml
     update_rate: 200
     ```

2. **Adjust joint A2 limits:**
   - `lbr_description/urdf/iiwa14/joint_limits.yaml`
     ```yaml
     A2:
       lower: -115
       upper: 115
     ```

3. **Refine MoveIt joint bounds:**
   - `lbr_moveit_config/iiwa14_moveit_config/config/joint_limits.yaml`
     ```yaml
     lbr_A2:
       has_velocity_limits: true
       min_position: -2.00
       max_position: 2.00
       max_velocity: 1.4835
       has_acceleration_limits: true
       max_acceleration: 10.0
     ```

4. **Modify robot URDF to include the OnRobot RG2 gripper:**
    At `lbr_description/urdf/iiwa14/iiwa14.xacro`, change the robot's starting pose and add the gripper.
  ```xacro
  <?xml version="1.0"?>
  <!-- top level -->
  <robot name="iiwa14"
      xmlns:xacro="http://www.ros.org/wiki/xacro">

      <!-- include the lbr iiwa macro -->
      <xacro:include filename="$(find lbr_description)/urdf/iiwa14/iiwa14_description.xacro" />

      <xacro:arg name="robot_name" default="lbr" />
      <xacro:arg name="mode" default="true" />
      <xacro:arg name="system_config_path" default="$(find lbr_description)/ros2_control/lbr_system_config.yaml" />

      <!-- KDL requires a link without inertia / Gazebo requires a connection to world link or robot
      will tipple https://github.com/lbr-stack/lbr_fri_ros2_stack/issues/230 -->
      <link name="world" />

      <!--joint
          between <robot_name>_world_link and robot_name_link_0-->
      <joint name="$(arg robot_name)_world_joint" type="fixed">
          <parent link="world" />
          <child link="$(arg robot_name)_link_0" />
          <origin xyz="0.0 0.0 1.586" rpy="-1.57 0 -0.524" /> <!-- right arm is 1.57 0 0.524 -->
      </joint>

      <!-- iiwa -->
      <xacro:iiwa14 robot_name="$(arg robot_name)" mode="$(arg mode)" system_config_path="$(arg system_config_path)" />

      <!-- onrobot -->
      <xacro:arg name="onrobot_type" default="rg2"/>
      <xacro:arg name="prefix" default=""/>
      <xacro:arg name="name" default="onrobot"/>

      <!-- Import the OnRobot macro -->
      <xacro:include filename="$(find onrobot_rg_gazebo)/urdf/onrobot_macro.xacro"/>

      <!-- Create OnRobot instance -->
      <xacro:onrobot onrobot_type="$(arg onrobot_type)" prefix="$(arg prefix)" />
      <joint name="$(arg prefix)onrobot_base_link_joint" type="fixed">
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <parent link="$(arg robot_name)_link_ee"/>
          <child link="$(arg prefix)onrobot_base_link"/>
      </joint>

  </robot>
  ```

5. **Add OnRobot Gazebo resources to path:**

   ```bash
   export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix onrobot_description)/share
   ```

---

##  Quick Start Checklist

| Step | Action                                               | Notes                                           |
| ---- | ---------------------------------------------------- | ----------------------------------------------- |
| 1    | Plug **Ethernet** from PC → KUKA **KONI (X66)** port | For robot control                               |
| 2    | Power OnRobot compute box & connect via switch       | For gripper control                             |
| 3    | Set PC static IP                                     | `192.168.11.2/24`                               |
| 4    | Ping devices                                         | Robot: `172.31.1.147` · OnRobot: `172.31.1.4`   |
| 5    | Launch **LBRServer** on SmartPad                     | FRI send: `10ms` · Mode: `POSITION_CONTROL`     |
| 6    | Start ROS 2 drivers                                  | See below                                       |
| 7    | Test gripper open/close                              | Use `/onrobot/finger_width_controller/commands` |

> ⚠️ Always **hold the SmartPad dead-man switch** and **play buttons** during motion.
> ⚠️ **Check workspace** before executing MoveIt targets — collisions are possible.

---

## Step-by-Step Hardware Setup


### 1. Verify Ethernet Connection

```bash
sudo ip addr add 172.31.1.148/24 dev eth0
sudo ip link set eth0 up
ping 172.31.1.147
```

If no reply:

* Recheck cable or SmartPad state.
* Restart interface:
  `sudo ip link set eth0 down && sudo ip link set eth0 up`

### 2. Launch LBRServer on SmartPad

* FRI send period → `10 ms`
* FRI control mode → `POSITION_CONTROL`
* Client command mode → `POSITION`
* Client IP → your PC (`192.168.11.2`)

### 3. Run ROS 2 Drivers

```bash
ros2 launch lbr_bringup hardware.launch.py moveit:=true model:=iiwa14
ros2 launch lbr_bringup move_group.launch.py mode:=hardware model:=iiwa14 rviz:=true
```

### 4. OnRobot Driver

```bash
ros2 launch onrobot_driver onrobot_control.launch.py \
    onrobot_type:=rg2 connection_type:=tcp ip_address:=172.31.1.4
```

### 5. Test Gripper Control

```bash
ros2 topic pub --once /onrobot/finger_width_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [0.05]}"
```

---

## MoveIt 2 Hardware Operation

MoveIt 2 Hardware Operation

[📖 Official MoveIt Docs](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_moveit_cpp/doc/lbr_moveit_cpp.html)

```bash
ros2 launch lbr_bringup hardware.launch.py moveit:=true model:=iiwa14
ros2 launch lbr_bringup move_group.launch.py mode:=hardware model:=iiwa14 rviz:=true
ros2 launch lbr_moveit_cpp hello_moveit.launch.py mode:=hardware model:=iiwa14
```

> 💡 Ensure `client_command_mode: position` and `update_rate: 100` in configuration YAMLs.

---

## OnRobot Gripper Setup

### Connection

| Device        | IP           | Interface      |
| ------------- | ------------ | -------------- |
| Left arm RG2  | `172.31.1.4` | Wi-Fi (Marlab) |
| Right arm RG2 | `172.31.1.3` | Wi-Fi (Marlab) |

### Driver Launch

```bash
ros2 launch onrobot_driver onrobot_control.launch.py \
    onrobot_type:=rg2 connection_type:=tcp ip_address:=172.31.1.4
```

### Manual Control

```bash
ros2 topic pub --once /onrobot/finger_width_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [0.05]}"
```

---

## PTU-46-17.5 (Pan-Tilt Unit) Setup

> Controlled via USB-to-RS232 on Ubuntu 22.04.

1. Identify port:

   ```bash
   dmesg | grep ttyUSB
   ```
2. Add permissions:

   ```bash
   sudo usermod -a -G dialout $USER
   ```
3. Connect via serial:

   ```bash
   screen /dev/ttyUSB0 9600
   ```
4. Example commands:

   | Action        | Command   |
   | ------------- | --------- |
   | Pan query     | `PP`      |
   | Tilt query    | `TP`      |
   | Move pan +10° | `PP 593`  |
   | Move tilt −5° | `TP -297` |
   | Reset         | `R`       |

> ~59.3 counts = 1°. Always end with a carriage return (`\\r`).

---

## 🤖 Hardware Setup

### 1. Connect robot

* Connect to **KONI port** and **Marlab WiFi**
* On the **SmartPad**, launch **LBRServer** with:

  * FRI send period: `10 ms`
  * FRI control mode: `POSITION_CONTROL`
  * Client command mode: `POSITION`
  * Client IP: your PC’s IP (e.g. `192.168.11.2`)

### 2. Start robot and drivers

* **Robot launch (with MoveIt 2 & RViz)**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab marlab_hardware.launch.py \
    moveit:=true mode:=hardware model:=iiwa14 rviz:=true
  ```

* **OnRobot driver (real gripper)**

  ```bash
  ros2 launch onrobot_driver onrobot_control.launch.py \
    onrobot_type:=rg2 connection_type:=tcp ip_address:=172.31.1.4
  ```
  172.31.1.3 right gripper

### 3. Control nodes

* **Log robot state**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab state_logger.launch.py \
    mode:=hardware model:=iiwa14
  ```

* **Joint control**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab joint_control.launch.py \
    mode:=hardware model:=iiwa14
  ```

* **Cartesian path planning**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab cartesian_path_planning.launch.py \
    mode:=hardware model:=iiwa14
  ```

* **Find object poses**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab find_object_poses.launch.py \
    mode:=hardware model:=iiwa14
  ```

* **Low-level control**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab low_level_control_node.launch.py \
    mode:=hardware model:=iiwa14
  ```

* **High-level control (dnf-architecture)**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab high_level_control_node.launch.py
  ```
  ```bash
  ros2 topic pub /scene_objects kuka_lbr_iiwa14_marlab/msg/SceneObjects "{
    objects: [
      {type: 's', position: 10.0},
      {type: 's', position: 50.0},
      {type: 's', position: 30.0}
    ]
  }"
  ```

* **Vision processing node**
  
  ```bash
  ros2 run kuka_lbr_iiwa14_marlab  vision_processing_node.py 
  ```


---
## Simulation Setup

### 1. Start MoveIt + Gazebo

```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka_move_group_env.launch.py \
  moveit:=true model:=iiwa14 use_sim_time:=true rviz:=true mode:=gazebo
```

### 2. Fake OnRobot Driver

```bash
ros2 launch onrobot_driver onrobot_control.launch.py \
  onrobot_type:=rg2 connection_type:=tcp use_fake_hardware:=true
```

### 3. Test Nodes

| Purpose                 | Command                                                                                           |
| ----------------------- | ------------------------------------------------------------------------------------------------- |
| Joint control           | `ros2 launch kuka_lbr_iiwa14_marlab joint_control.launch.py mode:=gazebo model:=iiwa14`           |
| Cartesian path planning | `ros2 launch kuka_lbr_iiwa14_marlab cartesian_path_planning.launch.py mode:=gazebo model:=iiwa14` |
| State logger            | `ros2 launch kuka_lbr_iiwa14_marlab state_logger.launch.py mode:=gazebo model:=iiwa14`            |

---

## 📚 References

* [LBR Stack Docs](https://lbr-stack.readthedocs.io/)
* [MoveIt Hardware Setup](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_moveit/doc/lbr_moveit.html)
* [OnRobot ROS 2 Driver](https://github.com/Jgocunha/OnRobot_ROS2_Driver)

---

> 🧾 **Tested Environment:** Ubuntu 22.04 · ROS 2 Humble · MoveIt 2 · Gazebo 11