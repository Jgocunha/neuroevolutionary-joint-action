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
   ```xml
   <xacro:include filename="$(find onrobot_rg_gazebo)/urdf/onrobot_macro.xacro"/>
   <xacro:onrobot onrobot_type="rg2" prefix=""/>
   <joint name="onrobot_base_link_joint" type="fixed">
       <parent link="lbr_link_ee"/>
       <child link="onrobot_base_link"/>
       <origin xyz="0 0 0" rpy="0 0 0"/>
   </joint>
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

Simulation Setup

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