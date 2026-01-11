# Elbow-Angle-Tracking-Using-Wearable-IMUs

This project developed a wearable elbow angle estimation system using two IMU sensors and an ESP32 microcontroller, communicating via Micro-ROS with ROS 2. After comparing three sensor fusion techniques, the Madgwick filter was selected for its superior accuracy and stability. The system was validated through both controlled flat surface experiments and human arm trials.

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
| `Container is not running` | Run `sudo docker start microros_jazzy4` |
