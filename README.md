# See Original Work
https://github.com/roman2veces/unitree_ros2_to_real

# Introduction
This package can send control commands to the Unitree A1 robot from ROS2. 
This package is able also to collecting data from the IMU sensor inside the Unitree 
and give the info State from the Robot
This version is suitable for unitree_legged_sdk v3.2.1.

# Dependencies:
(this should be all installed by the Dockerfile)

* [unitree_legged_sdk v3.2 (fork)](https://github.com/roman2veces/unitree_legged_sdk)
* [ros2_unitree_legged_msgs (fork)](https://github.com/roman2veces/ros2_unitree_legged_msgs)
* [lcm](https://github.com/lcm-proj/lcm/archive/refs/tags/)

# Environment
Tested in Ubuntu 20.04 using ros2 foxy. It can be also used in MacOS or Windows but with usb devices limited support, so you couldn't drive the robot with a usb controller but you could with the keyboard.

# Build (using Docker)
Start by cloning this repo:
```
git clone https://github.com/marzima/unitree_ros2_to_real_main.git
```

if this is the first time you use this docker image, run the following commands: 

```
cd unitree_ros2_to_real_main
docker build -t <image name> .

# The argument: -v /dev/:/dev/ allows us to access to the usb controller in Linux
#If you want to using Joystick to collecting data, you have to run 2 different docker
#container:
# 1 Privileged --> Using for the Joystick only
# 2 Hosting Container --> To be able to recieve and send data from Master (Running in the Host Machine) 
docker run --name <container name> --privileged -v /dev/:/dev/ -it <image name>
docker run --net=host -it <container name2>

exit 
```

# Run the package inside the OS Unitree (This using a TX2 NVIDIA)
First, make sure that the A1 is on and standing up correctly. Then, connect your computer to 
the robot wifi. This robot has inside ROS melodic. To recieve all the topics from Unitree
inside your hosting machine (I have ROS-noetic inside), make sure to run a Master inside your 
Hosting Machine (follow this step):
https://razbotics.wordpress.com/2018/01/23/ros-distributed-systems/
```
roscore #launch inside your Hosting Machine
```

Open a terminal and connecting inside the robot OS:
```
 ssh unitree@192.168.123.12
 roslaunch realsense2_camera opensource_tracking.launch #driver for RGB-D Camera
 #Open another terminal and run (For Velodyne16 Lidar):
 ssh unitree@192.168.123.12
 roslaunch velodyne_pointcloud VLP16_points.launch
```
# Inside the Privileged Container (Joy, IMU and STATE)
**Terminal 1:**

So you have to laucnh this driver inside your docker container:
If you want also to collecting data you have to connect the usb controller to your computer.
- joy_control.launch.py - to control the robot in any mode with a usb controller (only 1 terminal)

```
docker start -i <container name>

ros2 launch unitree_ros2_to_real_main joy_control.launch.launch.py
```
# Inside the HOST-Container
MAKE SURE YOU HAVE ROS-BRIDGE INSIDE CONTAINER (OTHERWISE YOU HAVE TO INSTALL IT with sudo apt-get install)
**Terminal 1:**
```
docker exec -it <container name> bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics #Now you can see all the topics if you want
```
**Terminal 2:**
If you want to record all the data in a ROS bag, exec the host container to recieve all topic from the robot:
```
ros2 bag record --all 
```

# How to read robot state?
In other terminal running the docker container you could see the [high state robot data](https://github.com/roman2veces/ros2_unitree_legged_msgs/blob/master/msg/HighState.msg).
```
docker exec -it <container name> bash

# if you want to see all the high state data 
ros2 topic echo /state

# if you want to only the IMU data (make sure that the launch parameter using_imu_publisher is true)
ros2 topic echo /imu
```

This data can be use it by reading the variable high_state_ros in the [src/twist_driver.cpp](https://github.com/roman2veces/unitree_ros2_to_real/blob/main/src/twist_driver.cpp)

# Important points

- When building the docker image in the robot, the robot computer has a arm64 architecture. So, make sure you change the environment variable UNITREE_PLATFORM to arm64 in the dockerfile before building the image or changing the environment variable in your current docker container. When running the docker container in your computer you should use the value amd64.

# Bugs and some problems in our A1

- Warning that we don't understand and we didn't try to fix yet (but everything works anyways): [lcm_server_3_2-1] Error! LCM Time out.

- If you don't see the usb controller values, for example if you use ros2 topic echo /joy, you should make sure that you connected de usb controller before starting the container.

- If you are not able to ping the robot computers from the container, make sure that you connect to the wifi before starting the container (in MacOS restarting the computer could be also necessary) 

