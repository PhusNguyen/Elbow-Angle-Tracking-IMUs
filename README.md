# Overview

A wearable system that estimates elbow flexion angle in real time from two IMUs, one on the upper arm and one on the forearm. An ESP32 publishes raw IMU data over micro-ROS to a ROS 2 Jazzy host, where a sensor fusion node estimates each segment's orientation and an angle estimation node computes the joint angle by modeling the arm as a two-link kinematic chain with D-H parameters.
Three fusion algorithms were implemented and benchmarked (Complementary filter, Madgwick, Extended Kalman Filter). Madgwick was selected for the best accuracy and stability. The system was validated in controlled flat-surface tests and on human arm trials.

# Hardware

| Component | Notes |
|-------|----------|
| ESP32 | micro-ROS client, serial transport (wifi transport will be updated in the next release) |
| 2xIMU BNO085 | upper arm and forearm placement |
| Straps / sleeve | wearable mounting |

# Repository structure
- MicroROS_Jazzy_ESP32/ — firmware for the ESP32 micro-ROS client
- src/ — ROS 2 packages (filter_bringup, angle_estimation)
- analysis_scripts/ — filter comparison and error analysis
- rosbag_file/ — recorded trials
- figures/ — plots and system diagrams

# Docker Setup Guide (Fedora 41)

## Prerequisites

- Docker installed on your system
- ROS 2 Jazzy base image (`ros:jazzy`)

## Initial Setup (First Time Only)

Build and create the Docker container:

```bash
sudo docker run -it --name microros_jazzy \
    --net=host \
    --privileged \
    -v "$PWD":/workspace \
    -v /dev:/dev \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY=$DISPLAY \
    ros:jazzy
```

## Running the Container

### 1. Start Docker daemon

```bash
sudo systemctl start docker
```

### 2. Enable GUI access for Docker

Grant Docker permission to access your X11 display (required for GUI applications like RViz):

```bash
sudo xhost +local:docker
```

To revoke permission when finished:

```bash
sudo xhost -local:docker
```

### 3. Start the container

```bash
sudo docker start microros_jazzy
```

To stop the container later:

```bash
sudo docker stop microros_jazzy
```

### 4. Open an interactive shell

```bash
sudo docker exec -it microros_jazzy /bin/bash
```

### 5. Source ROS 2 environment

```bash
source /opt/ros/jazzy/setup.bash
```

### 6. Source micro-ROS workspace

```bash
cd microros_ws
source install/local_setup.bash
```

## Running the Application

### Start micro-ROS agent

Replace `[device]` with your serial device (e.g., `/dev/ttyUSB0`):

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

### Launch ROS 2 nodes

Open a new terminal for each launch command (remember to enter the container and source the workspaces in each terminal).

**Sensor fusion:**

```bash
ros2 launch filter_bringup imu_madgwick_launch.py
```

**Angle estimation:**

```bash
ros2 launch angle_estimation angle_estimation.launch.py
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `failed to connect to docker API` | Run `sudo systemctl start docker` |
| `cannot open display` | Run `sudo xhost +local:docker` |
| `Container is not running` | Run `sudo docker start microros_jazzy` |
