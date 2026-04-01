# KUKA LBR and OnRobot Hardware & MoveIt Setup

This document explains how to set up a wired connection to the KUKA controller, verify connectivity, configure applications, and run MoveIt both with the robot and with OnRobot hardware.

Note: To send commands to the robot arm you need a wired connection. To send commands to the gripper you need to be connected to the MARLAB wi-fi.

---

## Quick Start Checklist

1. **Connect hardware**

   - Plug Ethernet cable from PC → **KUKA KONI (X66)** port.
   - Ensure OnRobot compute box is powered on & connected to the switch.

2. **Set PC IP (manually)**

   - Address: `192.168.11.2`
   - Netmask: `255.255.255.0`
   - Gateway: `192.168.11.2`

3. **Ping devices**

   - Robot: `ping 172.31.1.147`
   - OnRobot: `ping 172.31.1.4`

4. **Launch LBRServer on SmartPad**

   - FRI send period: `10 ms`
   - FRI control mode: `POSITION_CONTROL`
   - Client command mode: `POSITION`
   - IP: `192.168.11.2` (your PC’s IP)

5. **Run ROS2 drivers**

   ```bash
   # Robot driver
   ros2 launch lbr_bringup hardware.launch.py moveit:=true model:=iiwa14

   # MoveIt (with RViz)
   ros2 launch lbr_bringup move_group.launch.py mode:=hardware model:=iiwa14 rviz:=true

   # Hello MoveIt demo
   ros2 launch lbr_moveit_cpp hello_moveit.launch.py mode:=hardware model:=iiwa14
   ```

6. **Run OnRobot driver (optional)**

   ```bash
   ros2 launch onrobot_driver onrobot_control.launch.py \
       onrobot_type:=rg2 \
       connection_type:=tcp \
       ip_address:=172.31.1.4
   ```

7. **Test gripper open/close**

   ```bash
   ros2 topic pub --once /onrobot/finger_width_controller/commands \
   std_msgs/msg/Float64MultiArray "{data: [0.05]}"
   ```

⚠️ **Always hold the SmartPad control pad + play buttons during motion.**
⚠️ **Check workspace before sending MoveIt targets — collisions are possible.**

---

## 1. Hardware Setup

[Official Hardware Setup Documentation](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html)

### Wired Connection Settings

- **PC Ethernet (manual configuration):**

  - Address: `192.168.11.2`
  - Netmask: `255.255.255.0`
  - Gateway: `192.168.11.2`

- **KUKA KONI port:** Connect robot to the KONI (X66) port.

- **OnRobot IP:** `172.31.1.4`

---

### Verify Connection to the Robot

1. **Plug in Ethernet cable**
   Connect your PC’s Ethernet port to the **X66** port on the KUKA controller.

2. **Set PC’s Ethernet IP**
   Example configuration (adjust interface name accordingly):

   ```bash
   sudo ip addr add 172.31.1.148/24 dev eth0
   sudo ip link set eth0 up
   ```

3. **Ping the robot**

   ```bash
   ping 172.31.1.147
   ```

   Expected output:

   ```
   64 bytes from 172.31.1.147: icmp_seq=1 ttl=64 time=0.2 ms
   ```

   If no response:

   - Check cable connection.
   - Verify IP doesn’t conflict.
   - Ensure robot is fully booted (check SmartPad).
   - Restart network interface:

     ```bash
     sudo ip link set eth0 down
     sudo ip link set eth0 up
     ```

4. **(Optional) Confirm with ARP**

   ```bash
   arp -a
   ```

   Expected entry:

   ```
   ? (172.31.1.147) at 00:30:de:ad:be:ef [ether] on eth0
   ```

---

## 2. Configure & Launch LBRServer

On the **KUKA SmartPad**:

1. Launch the **LBRServer** application.
2. Configure:

   - FRI **send period**: `10 ms`
   - IP address: your PC’s IP (e.g., `192.168.11.2`)
   - FRI control mode: `POSITION_CONTROL` or `JOINT_IMPEDANCE_CONTROL`
   - FRI client command mode: `POSITION`

⚠️ **Note:** You must hold the **control pad** and **play buttons** for motion.

---

## 3. Running the Robot with MoveIt

[MoveIt Hardware Documentation](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_moveit_cpp/doc/lbr_moveit_cpp.html)

### Client-Side Configurations

- In `lbr_system_config.yaml`:
  `client_command_mode: position`
- In `lbr_controllers.yaml`:
  `update_rate: 100`

### Launching

1. **Run the robot driver**

   ```bash
   ros2 launch lbr_bringup hardware.launch.py moveit:=true model:=iiwa14
   ```

2. **Run MoveIt**

   ```bash
   ros2 launch lbr_bringup move_group.launch.py mode:=hardware model:=iiwa14 rviz:=true
   ```

3. **Run the Hello MoveIt demo**

   ```bash
   ros2 launch lbr_moveit_cpp hello_moveit.launch.py mode:=hardware model:=iiwa14
   ```

   ⚠️ Careful — the target position may cause collisions.

---

## 4. OnRobot Setup

### Connection

1. Ensure the OnRobot compute box is powered on.
2. Connect via Ethernet to the switch.
3. Connect your PC to the MARLAB Wi-Fi.
4. Ping the OnRobot box:

   ```bash
   ping 172.31.1.4
   ```

### Running the Driver

```bash
ros2 launch onrobot_driver onrobot_control.launch.py \
    onrobot_type:=rg2 \
    connection_type:=tcp \
    ip_address:=172.31.1.4
```

### Test: Open/Close Gripper

1. Start the OnRobot driver (as above).
2. Launch control:

   ```bash
   ros2 launch kuka_lbr_iiwa14_marlab onrobot_rg2_control.launch.py
   ```

3. Command gripper width:

   ```bash
   ros2 topic pub --once /onrobot/finger_width_controller/commands \
   std_msgs/msg/Float64MultiArray "{data: [0.05]}"
   ```

---

## 5. References

- [lbr_fri_ros2_stack GitHub](https://github.com/lbr-stack/lbr_fri_ros2_stack)
- [MoveIt via RViz (Hardware)](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_moveit/doc/lbr_moveit.html#moveit-via-rviz-hardware)

---

## 6. Other preliminary steps (should already be done)

### KUKA LBR iiwa 14

[lbr_fri_ros2_stack docs hardware setup](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html)

1. You will need to be connected via ethernet to the robot's controller KONI port;
2. Set your PC's IP address to 192.168.11.2, netmask 255.255.255.0 and gateway 192.168.11.2;
3. KUKA left arm IP address on the x66 port is 172.31.1.148, the right arm is 172.31.1.147;
4. The KONI port IP address was set via the Sunrise Workbench application to 192.168.1.100; LEFT ARM
4. The KONI port IP address was set via the Sunrise Workbench application to 192.168.1.100; RIGHT ARM
5. When following the `Install applications to the robot` instructions on the lbr-stack docs, when cloning the `fri` repository checkout to branch `ros2-fri1.15`;
6. When following the `Install applications to the robot` instructions on the lbr-stack docs, at the end you will have to edit the `LBRServer.java` file manually to set the IP addresses of your machine accordingly (192.168.1.2 if using the KONI port).

## OnRobot RG2 or RG6

1. You have to be connected to the MARLab wi-fi;
2. OnRobot's controller IP address of the left arm is 172.31.1.4., RIGHT ARM IS 172.31.1.3.
