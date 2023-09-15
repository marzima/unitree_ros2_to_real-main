# Introduction
This package can send control commands to the Unitree A1 robot from ROS2. 
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
docker run --name <container name> --privileged -v /dev/:/dev/ -it <image name>

# We couldn't install this package with the Dockerfile, so you have to do it manually:
sudo apt install -y ros-foxy-teleop-twist-keyboard 
exit 
```

# Run the package
First, make sure that the A1 is on and standing up correctly. Then, connect your computer to 
the robot wifi. You can also optionnally connect the usb controller to your computer if you want to driver the robot with a controller, but this only works on Linux. 

Then, you have currently 3 options to launch this package:
- joy_control.launch.py - to control the robot in any mode with a usb controller (only 1 terminal)
- position_control.launch.py - to control the robot in position mode (yaw, pitch, roll) with the keyboard (2 terminals)
- walking_control.launch.py - to control the robot in walking mode with the keyboard (2 terminals)

**Terminal 1:**
```
docker start -i <container name>

# Here you can choose between joy_control.launch.py, position_control.launch.py or 
# walking_control.launch.py
ros2 launch unitree_ros2_to_real joy_control.launch.py
```

Only if you took position_control.launch.py or walking_control.launch.py, you will need the next 
terminal to launch a twist driver

**Terminal 2:**
```
docker exec -it <container name> bash

# I strongly recommend to change the speed when using the teleop_twist_keyboard package
# you will see the indications to do it after executing the next command
ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

