
---

# Enterprise Isaac Sim + ROS2 + Nav2 Full Environment

[![Docker Build](https://img.shields.io/badge/Docker-Build%20Passing-green)](https://hub.docker.com/r/newbiekd/isaac_sim_lab_ros2_nav2)
[![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-4.5.0-blue)](https://developer.nvidia.com/isaac-sim)
[![Isaac Lab](https://img.shields.io/badge/Isaac%20Lab-2.1.0-blue)](https://isaac-sim.github.io/IsaacLab/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)](https://docs.ros.org/en/humble/)

Modularize Isaac Container & Improvements Based on j3soon/ros2-essentials
* Refactored the Docker environment to support modular installation scripts for Isaac Sim and related components, enabling flexible extension and easier maintenance.
* Based on the original repository by [j3soon/ros2-essentials](https://github.com/j3soon/ros2-essentials/tree/main) with the following enhancements:
  * **Fixed**: Isaac Sim numpy version compatibility issue.
  * **Fixed**: Multi-GPU detection and initialization issues.
  * **Added**: Pre-installation of required machine learning libraries (e.g., torch, etc.) for Isaac Sim & IsaacLab.
  * **Added**: Nav2 package installation for ROS2 navigation support.
  * **Added**: X11 VNC support for remote GUI access.
  * **Added**: Default user password set to `0000` for easier initial access.
  * **Changed**: Removed `sudo` privileges from the container user for better security and enterprise compliance.

## 📑 Table of Contents

* [Requirements](#requirements)
* [Project Introduction](#project-introduction)
* [File Structure](#file-structure)
* [Quick Start](#quick-start)
* [Complete Test Workflow](#complete-test-workflow)

  * [1. User Mode Startup Test](#1-user-mode-startup-test)
  * [2. X11VNC Remote Desktop Test](#2-x11vnc-remote-desktop-test)
  * [3. Isaac Sim Startup Test](#3-isaac-sim-startup-test)
  * [4. IsaacLab Feature Test](#4-isaaclab-feature-test)
  * [5. ROS2 Feature Test](#5-ros2-feature-test)
* [FAQ / Troubleshooting](#faq--troubleshooting)

## Requirements

* **Operating System**: Ubuntu 20.04+ / Windows 11 (WSL2) / macOS (Docker Desktop)
* **Hardware**:

  * NVIDIA GPU (CUDA 11.8+ supported)
  * Memory: 16GB+ (32GB recommended)
  * Storage: 50GB+ available space
* **Software**:

  * Docker Engine 20.10+
  * Docker Compose 2.0+
  * NVIDIA Container Toolkit
* **Network**: Enterprise environment must allow access to Docker Hub and NVIDIA NGC

## Project Introduction

This project provides a **fully tested and enterprise-ready** Docker environment integrating Isaac Sim, ROS2, and Nav2.

### 🚩 Key Features

* ✅ **User Mode Startup**: Supports non-root user mode for enhanced security in enterprise environments
* ✅ **X11VNC Remote Desktop**: Built-in VNC server for GUI access via remote desktop
* ✅ **GPU Acceleration**: Full NVIDIA GPU support with CUDA computation
* ✅ **Integrated Stack**: Isaac Sim 4.5.0 + Isaac Lab 2.1.0 + ROS2 Humble + Nav2
* ✅ **Enterprise Friendly**: Proxy/firewall support, custom UID/GID, and other enterprise needs

### 📦 Included Components

* **Isaac Sim 4.5.0** - NVIDIA’s 3D robotics simulation platform
* **Isaac Lab 2.1.0** - Robot learning and training framework
* **ROS2 Humble** - Robot Operating System 2
* **Nav2** - ROS2 navigation stack
* **X11VNC** - Remote desktop service
* **CUDA Toolkit** - GPU computation support

## 📁 File Structure

```
corporate_docker/
├── README.md                               # This documenttions
└── docker/
    ├── Dockerfile                          # Main Docker build file
    ├── compose.yaml                        # Docker Compose config
    ├── cyclonedx.xml                       # ROS2 DDS config
    ├── .bashrc                             # Bash config and aliases
    └── modules/                            # Modular install scripts
        ├── install_ros.sh                  # ROS2 Humble + Nav2
        ├── install_x11_opengl_vulkan.sh    # Graphics support
        ├── install_cuda_toolkit.sh         # CUDA toolkit
        ├── install_isaac_sim.sh            # Isaac Sim 4.5.0
        ├── install_isaac_lab.sh            # Isaac Lab 2.1.0
        ├── install_isaac_ros.sh            # Isaac ROS
        ├── install_vnc.sh                  # VNC package install
        ├── start_xvfb_vnc.sh               # VNC startup script
        ├── stop_xvfb_vnc.sh                # VNC shutdown script
        └── cleanup_vnc.sh                  # VNC cleanup script
```

## 🚀 Quick Start

### 1. Build the Image

```bash
cd corporate_docker/docker
docker compose build
```

### 2. Start the Container (Development Mode)

```bash
xhost +local:docker # For X11 forwarding (optional)
docker compose up -d
```

### 3. Enter the Container

```bash
docker compose exec h1-ws bash

```
or
```bash
docker exec -it isaac-workspace bash

```

## 🧪 Complete Test Workflow

**This project has been fully tested in User Mode (non-root), with GPU, X11VNC, ROS2, Isaac Sim, and IsaacLab—all features verified and working.**

### 1. User Mode Startup Test

Start the container in non-root user mode to meet enterprise security requirements:

```bash
cd /path/to/isaac_ros2_nav2/corporate_docker/docker

# Start container in pure User Mode (UID:1017)
docker run --rm -it \
  --gpus all \
  --runtime=nvidia \
  --network=host \
  -e HOME=/home/user \
  -w /home/user \
  newbieKD/isaac_sim_lab_ros2_nav2:3.2.0 bash

```

**Checklist:**

* ✅ Non-root user startup works
* ✅ GPU devices are accessible
* ✅ Network is functional
* ✅ Permissions are correctly set

**Notes:**

* Check your user UID (run `id -u`) and replace `1017:1017` as needed
* `--network=host` ensures VNC and other services work

### 2. X11VNC Remote Desktop Test

Test the X11VNC remote desktop feature for GUI access:

```bash
# In the container
vnc-display
start-xvfb-vnc

# Check VNC status
vnc-user-status

# (optional) Stop VNC service
stop-xvfb-vnc
```

**Validate VNC GUI:**

```bash
# Test ROS2 GUI tool
ros2 run rviz2 rviz2

# Test other GUI apps
xclock
# or
xeyes
```

**VNC Connection Info:**

* **Address:** `<Host IP>:5900`
* **Password:** `0000` (default)
* **Display:** `:1` (1920x1080)

**Notes:**

* VNC uses virtual display (Xvfb), no physical monitor needed
* All VNC operations run under user privileges
* If connection fails, check firewall rules

### 3. Isaac Sim Startup Test

Test both Isaac Sim startup modes to ensure 3D simulation runs correctly:

#### Streaming Mode

```bash
cd ~/isaacsim
./isaac-sim.streaming.sh
```

#### Standard Mode

```bash
cd ~/isaacsim
./isaac-sim.sh
```

**Checklist:**

* ✅ Isaac Sim 4.5.0 starts successfully
* ✅ GPU acceleration works
* ✅ 3D rendering functions normally
* ✅ WebRTC streaming (in streaming mode)

**Notes:**

* Streaming mode supports Isaac Sim WebRTC Streaming Client
* Standard mode GUI is accessible via VNC
* First launch may take longer due to data loading

### 4. IsaacLab Feature Test

Test IsaacLab RL training environment:

```bash
cd ~/IsaacLab

# Run H1 humanoid robot training task
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Rough-H1-v0 \
  --headless \
  --livestream 2

# NOTE: Due to known issues in IsaacLab 2.1.0, "--livestream 2" may not work unless train.py is modified.
# You can verify training works with:
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task Isaac-Velocity-Rough-H1-v0 \
  --headless
```

**WebRTC Connection:**

* Streaming mode supports Isaac Sim WebRTC Streaming Client
* Real-time training observation supported

**Checklist:**

* ✅ IsaacLab 2.1.0 loads correctly
* ✅ RL environment initializes properly
* ✅ GPU-accelerated training works
* ✅ WebRTC streaming displays as expected

**Notes:**

* Use `--headless` for headless/server environments
* `--livestream 2` enables WebRTC streaming
* Training will generate checkpoints, ensure you have enough storage

### 5. ROS2 Feature Test

Test ROS2 Humble and Nav2 navigation stack:

#### Important

> **⚠️ The container does NOT include the ROS2 workspace by default; you must copy it in manually.**

#### Copy ROS2 Workspace

```bash
# Copy your local workspace into the container
docker cp <your_workspace_path>/src <container_name>:/home/ros2-essentials/humble_ws/src

# Example:
docker cp /path/to/JDP_code/humble_ws/src <container_name>:/home/ros2-essentials/humble_ws/src
```

#### ROS2 Setup & Test

```bash
# Enter ROS2 workspace
cd /home/ros2-essentials/humble_ws/

# Build all packages
colcon build

# Set environment
source /home/ros2-essentials/humble_ws/install/setup.bash

# Build specific package
colcon build --packages-select humanoid_navigation
source install/setup.bash

# Launch navigation service
ros2 launch humanoid_navigation h1_policy_improved.launch.py
```

#### Basic ROS2 Functionality

```bash
# Check ROS2 installation
ros2 --version

# List available nodes
ros2 node list

# List topics
ros2 topic list

# Test publisher/subscriber
ros2 run demo_nodes_cpp talker
# (in another terminal)
ros2 run demo_nodes_cpp listener
```

**Checklist:**

* ✅ ROS2 Humble is installed and configured
* ✅ Nav2 stack loads correctly
* ✅ Colcon build system works
* ✅ Custom ROS2 packages build and run as expected

**Notes:**

* ROS2 workspace must be copied in manually; the container does NOT ship with project code
* Building may take time; ensure sufficient resources
* Some navigation features may require additional sensor data or maps

---

## 🔧 Built-in Command Aliases

The container provides the following handy commands:

### VNC-Related

* `start-xvfb-vnc` - Start X11VNC service
* `stop-xvfb-vnc` - Stop X11VNC service
* `cleanup-vnc` - Cleanup VNC files
* `vnc-user-status` - Check user VNC process status
* `vnc-user-logs` - View VNC logs
* `vnc-display` - Set display for VNC (:1)
* `x11-display` - Restore X11 forwarding display

### System Information

* `gpu-info` - Show GPU information
* `system-info` - Show system info

For detailed VNC usage, see: [VNC\_README.md](./VNC_README.md)

## FAQ / Troubleshooting

### Q1: Permission error when starting container

**Problem:**
`docker: Error response from daemon: failed to create task for container: failed to create shim task`

**Solution:**

```bash
# Check your UID
id -u

# Start with the correct UID:GID
docker run --user $(id -u):$(id -g) ...
```

### Q2: VNC connection fails

**Problem:** Cannot connect to VNC service (port 5900)

**Solution:**

```bash
# Check if VNC is running
vnc-user-status

# Check if port is occupied
netstat -tlnp | grep :5900

# Restart VNC
stop-xvfb-vnc
start-xvfb-vnc
```

### Q3: GPU unavailable

**Problem:** Isaac Sim cannot use GPU acceleration

**Solution:**

```bash
# Check NVIDIA driver
nvidia-smi

# Ensure Docker is started with
--gpus all --runtime=nvidia
```

### Q4: X11 forwarding issues

**Problem:** GUI apps do not display

**Solution:**

```bash
# On host, allow X11 forwarding
xhost +local:docker

# In container, set DISPLAY correctly
export DISPLAY=:0  # for X11 forwarding
# or
vnc-display      # for VNC display
```

### Q5: File mount permission issues

**Problem:** Mounted files or directories have incorrect permissions

**Solution:**

```bash
# On host, ensure correct permissions
sudo chown -R $(id -u):$(id -g) /path/to/mounted/folder

# Or set correct user in compose.yaml
user: "${UID:-1000}:${GID:-1000}"
```

## 🏢 Enterprise Environment Notes

### Firewall Settings

Open the following ports:

* **5900**: VNC remote desktop
* **8211**: Isaac Sim WebRTC streaming

### Security Considerations

* Always run services as a non-root user
* Change the default VNC password for production use
* It is recommended to use in an internal network; avoid exposing directly to the public internet

---

## 📞 Support & Contribution

If you have questions or suggestions, please submit an Issue or Pull Request.

**Maintainer**: Binghua Cai
**Last Updated**: September 2025

---
