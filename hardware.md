# Hardware Setup 

[lbr-stack-hardware-setup](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html)

## Verify Connection to the Robot

### 1. Plug in Ethernet cable

* Connect your PC’s Ethernet port to the **X66** port on the KUKA controller (usually the RJ45 on the front panel).

---

### 2. Set your PC’s Ethernet IP manually

You want to configure your Ethernet interface (usually `eth0` or `enpXsY`) to match the robot's subnet.

#### First, identify your interface:

```bash
ip link show
```

Find the one that's **not `lo`**, and is down or disconnected before plugging in the cable — typically something like `eth0`, `enp2s0`, etc.

#### Then, assign an IP in the same subnet:

```bash
sudo ip addr add 172.31.1.148/24 dev eth0
sudo ip link set eth0 up
```

Use a different IP than the robot (e.g., `172.31.1.148` if the robot is `172.31.1.147`).

---

### 3. Ping the robot

Once you've assigned the IP:

```bash
ping 172.31.1.147
```

You should see output like:

```
64 bytes from 172.31.1.147: icmp_seq=1 ttl=64 time=0.2 ms
```

If you get **no response**, check:

* Cable is plugged in tightly.
* Your IP is correct and not conflicting.
* Robot is fully booted and on (check SmartPad).
* Try restarting the network interface:

  ```bash
  sudo ip link set eth0 down
  sudo ip link set eth0 up
  ```

---

### 4. (Optional) Confirm with ARP

```bash
arp -a
```

You should see an entry like:

```
? (172.31.1.147) at 00:30:de:ad:be:ef [ether] on eth0
```

---

## Install Applications to the Robot

---

## lbr_moveit_cpp

[lbr-stack-moveit-hardware](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_demos/lbr_moveit_cpp/doc/lbr_moveit_cpp.html)

1. Client side configurations:

    - Configure the ```client_command_mode``` to ```position``` in ```lbr_system_config.yaml```..

    - Set the ```update_rate``` to ```100``` in ```lbr_controllers.yaml```.

2. Remote side configurations:
    - Launch the ```LBRServer``` application on the ```KUKA smartPAD```
    - Select
        - FRI ```send period```: ```10 ms```
        - IP address: ```your configuration```
        - FRI control mode: ```POSITION_CONTROL``` or ```JOINT_IMPEDANCE_CONTROL```
        - FRI client command mode: ```POSITION```

3. Run the robot driver:

```bash
ros2 launch lbr_bringup hardware.launch.py \
    moveit:=true \
    model:=iiwa14
```

4. Run MoveIt:

```bash
ros2 launch lbr_bringup move_group.launch.py \
    mode:=hardware \
    model:=iiwa7
```

5. Run the ```hello_moveit``` node:

```bash
ros2 launch lbr_moveit_cpp hello_moveit.launch.py \
    mode:=hardware \
    model:=iiwa14
```