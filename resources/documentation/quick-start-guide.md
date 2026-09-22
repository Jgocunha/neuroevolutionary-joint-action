
# quick guide

## kuka

ros2 launch kuka_lbr_iiwa14_marlab marlab_hardware.launch.py moveit:=true mode:=hardware model:=iiwa14 rviz:=true

ros2 launch kuka_lbr_iiwa14_marlab joint_control.launch.py mode:=hardware model:=iiwa14

ros2 launch kuka_lbr_iiwa14_marlab find_object_poses.launch.py mode:=hardware model:=iiwa14

## onrobot

ros2 launch onrobot_driver onrobot_control.launch.py \
    onrobot_type:=rg2 connection_type:=tcp ip_address:=172.31.1.3

ros2 topic pub --once /onrobot/finger_width_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [0.05]}"

## pan-tilt

sudo dmesg | grep ttyUSB

sudo usermod -a -G dialout $USER

screen /dev/ttyUSB0 9600

TP -800

## zed 2i

Go to directory src\tests\zed-2i\right_arm

python3 zed_online.py --dev /dev/video4 --size 2560x720 --fps 60 --fmt MJPG \
  --half right --mode both \
  --roi-objects 607,400,235,100 --roi-width-cm-objects 60 \
  --roi-hand    607,350,215,110  --roi-width-cm-hand 60 \
  --only-changes --change-th 6 --show


## putting it all together

ros2 launch kuka_lbr_iiwa14_marlab marlab_hardware.launch.py moveit:=true mode:=hardware model:=iiwa14 rviz:=true

ros2 launch kuka_lbr_iiwa14_marlab joint_control.launch.py mode:=hardware model:=iiwa14S

ros2 launch onrobot_driver onrobot_control.launch.py \
    onrobot_type:=rg2 connection_type:=tcp ip_address:=172.31.1.3

ros2 launch kuka_lbr_iiwa14_marlab low_level_control_node.launch.py mode:=hardware model:=iiwa14

ros2 launch kuka_lbr_iiwa14_marlab high_level_control_node.launch.py

ros2 run kuka_lbr_iiwa14_marlab vision_processing_node.py