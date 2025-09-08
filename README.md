# KUKA LBR iiwa 14 Control with ROS 2

This package demonstrates how to control the **KUKA LBR iiwa 14** robot in **joint space** and **Cartesian space (MoveIt 2)**.
It supports both **simulation (Gazebo + MoveIt 2)** and **hardware deployment** with an **OnRobot RG2 gripper**.

Tested on **Ubuntu 22.04** with **ROS 2 Humble**.

⚠️ Before hardware deployment please take a look at HARDWARE-SETUP.md.

---

## 📦 Dependencies

Before running, make sure you have installed:

* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* [lbr\_fri\_ros2\_stack](https://lbr-stack.readthedocs.io/)
* [marlab\_env\_description](https://github.com/Jgocunha/marlab_env_description) (Gazebo world files)
* [onrobot\_ros2\_driver](https://github.com/Jgocunha/OnRobot_ROS2_Driver)
* [onrobot\_rg\_gazebo](https://github.com/Jgocunha/onrobot_rg_gazebo)
* [MoveIt 2](https://moveit.picknik.ai/main/doc/getting_started/getting_started.html)
* Gazebo with ROS 2 integration

Direct changes to `lbr\_fri\_ros2\_stack` (this should not be done like this, but it is for now):
1. At `lbr_description/ros2_control/lbr_controllers.yaml`, increase `update_rate: 200`;
2. At `lbr_description/urdf/iiwa14/joint_limits.yaml`, decrease limits of joint A2 to -115 and 115.
3. At `lbr_moveit_config/iiwa14_moveit_config/config/joint_limits.yaml` 
   ```yaml
   lbr_A2:
    has_velocity_limits: true
    # use a *slightly tighter* bound than the mechanical/driver limit
    min_position: -2.00
    max_position: 2.00
    max_velocity: 1.4835298641951802
    has_acceleration_limits: true
    max_acceleration: 10.0
    ```
4. At `lbr_description/urdf/iiwa14/iiwa14.xacro`, change the robot's starting pose and add the gripper.
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
          <origin xyz="0.0 0.0 1.586" rpy="-1.57 0 -0.524" />
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

👉 Important: add the OnRobot Gazebo models to Ignition’s resource path:

```bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix onrobot_description)/share
```

---

## 🚀 Simulation Setup

### 1. Start simulation with MoveIt 2

```bash
ros2 launch kuka_lbr_iiwa14_marlab marlab_kuka_move_group_env.launch.py \
  moveit:=true model:=iiwa14 use_sim_time:=true rviz:=true mode:=gazebo
```

### 2. Start OnRobot driver (RG2 gripper, fake hardware)

```bash
ros2 launch onrobot_driver onrobot_control.launch.py \
  onrobot_type:=rg2 connection_type:=tcp use_fake_hardware:=true
```

⚠️ Sometimes this needs to be run **twice** if the gripper does not load correctly in RViz.

### 3. Test control nodes

* **Joint control**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab joint_control.launch.py \
    mode:=gazebo model:=iiwa14
  ```

* **Cartesian path planning**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab cartesian_path_planning.launch.py \
    mode:=gazebo model:=iiwa14
  ```

* **OnRobot control**

  ```bash
  ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
  ros2 launch kuka_lbr_iiwa14_marlab onrobot_rg2_control.launch.py
  ```

* **State logging**

  ```bash
  ros2 launch kuka_lbr_iiwa14_marlab state_logger.launch.py \
    mode:=gazebo model:=iiwa14
  ```

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
  }
  ```

  Note: The loaded imgui layout is located at `ros2_ws` or, if saved, at `dynamic-neural-field-composer/dynamic-neural-field-composer/resources/layouts\packaging task control architecture_layout.ini`.

### 4. Run controlled scenarios

* **Independent action**
  
  ```bash
  ros2 run kuka_lbr_iiwa14_marlab independent_action
  ```

* **Anticipatory Action**

  ```bash
  ros2 run kuka_lbr_iiwa14_marlab anticipatory_action
  ```

* **Collaboratice Preference Action**
  
  ```bash
  ros2 run kuka_lbr_iiwa14_marlab collaborative_preference_action
  ```


---

## 📚 References

* [lbr\_fri\_ros2\_stack Documentation](https://lbr-stack.readthedocs.io/)

---

## 🛠 Roadmap / To-Do

* [ ] Integrate Kinect or ZED for video capture
* [ ] Fix continuous dimensions of fields
* [ ] Re-test with updated architecture (dimension consistency)

