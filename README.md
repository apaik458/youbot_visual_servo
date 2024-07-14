# youbot_visual_servo
A simple 2-axis IBVS application on the KUKA youBot robot arm using MoveIt Servo and ArUco

Tested on Ubuntu 20.04 using ROS and MoveIt Noetic

<img src="youbot_edit.gif" width="750">

## Setup

### Create workspace
```
mkdir -p ~/youbot_ws/src && cd ~/youbot_ws/src
```

### Clone repos
Clone repo
```
git clone https://github.com/apaik458/youbot_visual_servo.git
```

Clone driver and ros-interface repos
```
git clone https://github.com/nssnikolas/kuka-youbot-driver.git
git clone https://github.com/nssnikolas/kuka-youbot-ros-interface.git
```

Clone description and moveit_config repos
```
git clone https://github.com/a2s-institute/youbot_description.git
git clone https://github.com/nssnikolas/kuka-youbot-moveit.git
```

Clone msg repos (take pr2_msgs from pr2_common and pr2_controllers_msgs from pr2_controllers)
```
git clone https://github.com/wnowak/brics_actuator.git
git clone https://github.com/PR2/pr2_common.git
git clone https://github.com/PR2/pr2_controllers.git
```

### Build and source workspace
```
cd ~/youbot_ws
catkin_make
source devel/setup.bash
```

## Launching Driver

### Configure ethernet connection
Change `EthernetDevice` field in `kuka-youbot-driver/config/youbot-ethercat.cfg` to youBot address
```
ip addr show
```
Change `youBotHasBase` parameter in `kuka-youbot-ros-interface/launch/youbot_driver.launch` to `false`

Change `robot_description` parameter in `kuka-youbot-ros-interface/launch/youbot_driver.launch` to `youbot_arm_only.urdf`

### Link to shared libraries
```
sudo ldconfig /opt/ros/noetic/lib
```

### Launch drivers
```
roslaunch youbot_driver_ros_interface youbot_driver.launch
```

## Running Scripts
```
python3 test_position.py
python3 test_velocity.py
```
