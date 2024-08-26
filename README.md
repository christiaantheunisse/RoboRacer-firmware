# RoboRacer-firmware

This repo contains the nodes that are necessary to get the basics of RoboRacer working. This includes nodes and launches to...

- [...] read the sensors: Lidar, wheel encoders and IMU
- [...] calculate the odometry from the wheel encoders
- [...] use SLAM
- [...] estimate the state using a general extended Kalman filter (EKF)
- [...] controls the motors

## Installation

Some parts of this code only work on the robot, so it is advised to directly install this on the Raspberry Pi.

### 1. Install Ubuntu

Install an Ubuntu image on a SD card (>= 32 GB). Make sure that it matches the ROS2 version you want to use. Instructions can be easily found online.

### 2. Install ROS

Install ROS2. Iron and Humble should both work with this code, but I expect that newer ROS versions should not give big problems. Instructions can be easily found online.

### 3. Install `pigpio`

Install the `pigpio` library with the instructions [here](https://abyz.me.uk/rpi/pigpio/download.html) . This library is to control the GPIO pins on the Raspberry Pi.

### 4. Install the code

Make a ROS workspace folder. E.g.:

    mkdir -p robot_ws/src

Clone this repository in the workspace with the following command. Make sure you are in the `src` folder of your workspace:

    cd robot_ws/src                                     # pwd is now robot_ws/src
    git clone <url-or-ssh-of-this-repo> .               # clone the contents directly into the folder
    !!! git submodule init                              # not sure 

Now we build the workspace. This should be done from the workspace folder `robot_ws`. You can try to build all the packages at once, but the Raspberry Pi will most likely freeze if you try this. Therefore, it is better to build one or a few packages at a time.

    cd ..                                               # pwd is now robot_ws
    colcon build --packages-select racing_bot_control
    colcon build --packages-select racing_bot_encoder
    colcon build --packages-select racing_bot_hat
    colcon build --packages-select racing_bot_imu
    colcon build --packages-select racing_bot_odometry
    colcon build --packages-select racing_bot_bringup

### 5. Run the code

Source the workspace (assume that the pwd is still `robot_ws`):

    source install/setup.bash

----Check on robot----- It is also possible to add this to your `.bashrc` (might be another file when connect through ssh), but this can also be found online. ()    

## The code

### `racing_bot_bringup`

ALSO INCLUDE EKF  -- racing_bot_bringup


### Extra

- FastDSS server