# onrobot_gripper_ros2
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)  ![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)  ![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=cplusplus&logoColor=white)

ROS2 Humble Package for OnRobot 2FG7 Gripper

## Description
This package provides a ROS2 Humble interface for controlling the OnRobot 2FG7 gripper.  
Communication with the gripper's control box is achieved using XML-RPC.

## Installation
Clone the repository and build the package in your ROS2 workspace:

```bash
# The xmlrpc-c library is used in this package
sudo apt update
sudo apt install libxmlrpc-c++8-dev libxmlrpc-core-c3-dev

# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/Bkyb/onrobot_gripper_ros2.git

# Build the package
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

Change the controller's NETWORK SETTINGS (refer to the OnRobot user manual).

![image](https://github.com/user-attachments/assets/999d0239-f33e-4373-82f2-67d1127f5bc3)


Add a network profile to Ubuntu as follows to establish the connection:

![image](https://github.com/user-attachments/assets/15aa1b98-0470-40d8-a852-e6d770f50572)

## Usage
Run the node to start the gripper service server:
```bash
# "192.168.137.101" is default IP setting
ros2 launch onrobot_2fg7 onrobot_launch.py ip:="192.168.137.101"
```
```bash
# Example
ros2 service call /grip_external onrobot_msgs/srv/GripExternal "{index: 0, width: 40.0, force: 20, speed: 10, is_wait: true}" 
```
To use another service, check the service files in the **onrobot_msgs** package.

