# The Tourminator

## What Is This

The Tourminator is my computer engineering capstone project. It is an autonomous indoor mapping and virtual tour robot built on a Raspberry Pi 4, running ROS Noetic. The robot drives itself around an indoor space, builds a map of the environment using SLAM, navigates autonomously to photo waypoints it calculates on its own, and captures 360 degree photos at each one. The end goal is a robot that can walk into a building, map it, and produce everything needed for a virtual tour without any human intervention after the initial launch.

You can watch the final demo here: https://www.youtube.com/watch?v=s-Hn-MnkyMg

## My Role

My responsibilities covered two major areas that I had to get working together: the physical hardware integration and the autonomous navigation and mission logic.

On the hardware side, I integrated every component into a working system. The final robot was built on an aluminum extrusion frame and included the RPLidar A1M8 for laser scanning, a GY-91 IMU (MPU6500) for orientation sensing, a 360 degree camera, a pair of goBILDA motors driven through GPIO on the Raspberry Pi using gpiozero, and the Raspberry Pi 4 itself acting as the main compute unit. Getting all of these talking to each other reliably through ROS required writing custom bridge nodes for each one. The motor bridge handles differential drive kinematics and odometry estimation from commanded velocities. The IMU node reads raw accelerometer and gyroscope data over I2C, calibrates gyroscope bias on startup, and publishes sensor data in the ROS IMU message format. The camera bridge manages the camera, handles capture triggering from the mission planner, saves images with pose metadata as JSON, and falls back to a simulation mode if the hardware is unavailable. The LiDAR runs through the standard rplidar ROS driver but feeds into a custom angular filter node that strips out the rear half of the scan, which was needed because the camera mount was causing false obstacle detections.

On the software and navigation side, the entire ROS navigation stack was configured and tuned from scratch for this specific robot. This included setting up gmapping for SLAM with parameters tuned for the RPLidar A1 and the robots motion characteristics, configuring move_base with both a global and local costmap, tuning the DWA local planner, and setting up explore_lite for frontier based autonomous exploration. The robots full URDF was defined including all sensor frames, wheel geometry, and TF transforms so that the navigation stack had an accurate model of the physical robot to work with. A robot localization configuration was also written to fuse wheel odometry with IMU data through an EKF.

The centerpiece of the mission software is the indoor_mapper node, which acts as the high level mission coordinator. It monitors exploration progress by watching frontier counts and map coverage, decides when the space has been sufficiently mapped, then transitions the robot into photography mode where it generates an optimized grid of photo waypoints from the completed map, navigates to each one using move_base, triggers the 360 degree camera capture sequence, and then returns the robot to its starting position. The whole pipeline runs without any human input once launched.

A full suite of hardware tests was also written for validating each component independently before integration, along with a LiDAR calibration tool for measuring exact robot boundary distances, IMU axis orientation tools, a behavioral debug monitor that gives structured real time insight into the robots decision making, and shell script based system monitors for checking topic health and node status during development.

## How It All Works Together

The system is split into two ROS packages: hardware_interface handles all the low level hardware bridging, and indoor_mapping_robot handles navigation, SLAM, and mission logic.

When you launch the robot, hardware.launch starts everything in sequence. The RPLidar node starts publishing laser scans on /scan. The motor_bridge node starts listening for velocity commands on /cmd_vel_nav, converts them to differential drive wheel speeds, drives the GPIO pins, and publishes odometry on /odom. The IMU node reads the MPU6500 over I2C and publishes on /imu. Static TF transforms are published to connect all the sensor frames to base_link based on their physical mounting positions, and the robot state publisher handles the rest of the TF tree from the URDF.

Then navigation.launch starts the rest of the stack. The laser filter node takes /scan and produces /scan_filtered which is the forward hemisphere only. gmapping subscribes to /scan_filtered and /odom and starts building a map published on /map. The robot localization EKF fuses /odom and /imu to produce a better odometry estimate. move_base uses the map and the filtered scan to plan paths and sends velocity commands to /cmd_vel_nav. explore_lite sends navigation goals to move_base to drive frontier exploration.

The indoor_mapper node sits on top of all of this. It watches /map to track coverage percentage and frontier counts, monitors the TF tree to track the robots pose, and waits for the exploration phase to reach completion either by coverage threshold, frontier exhaustion, or timeout. When it decides exploration is done it cancels the explore_lite goals, computes a grid of photo waypoints from the free space in the map, optimizes the visiting order by nearest neighbor, and then drives to each one using move_base action goals. At each waypoint it triggers the camera to capture a 360 degree photo, then moves to the next. Once all waypoints are done it navigates back to the starting position and saves the final map.

## Stack

ROS Noetic on Ubuntu 20.04, running on a Raspberry Pi 4. Python 3 for all nodes. gpiozero for motor GPIO control. smbus for IMU I2C communication. OpenCV for camera capture. gmapping for SLAM. move_base and DWA planner for navigation. explore_lite for frontier exploration. robot_localization for EKF sensor fusion.

## Running It

roslaunch hardware_interface hardware.launch
roslaunch indoor_mapping_robot navigation.launch enable_exploration:=true
roslaunch indoor_mapping_robot indoor_mapper.launch

Or use the convenience script at the root of the repo which starts hardware and navigation together and handles shutdown cleanly.

./start_robot.sh
