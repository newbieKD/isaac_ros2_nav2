# Isaac Sim, Isaac Lab, ROS2, and Nav2 Integration

## Introduction

This project integrates Isaac Sim, Isaac Lab, ROS2 (Humble), and Nav2 to facilitate reinforcement learning training and path planning for humanoid robots. It provides a Docker containerized environment for ease of setup and deployment.

## Prerequisites

*   Docker
*   NVIDIA Container Toolkit
*   RealVNC Viewer (optional, for remote GUI access)

## Installation

### Option 1: Build Docker Image

1.  Navigate to the `isaac_ros2_nav2` directory.
2.  Build the Docker image:

    ```bash
    docker build -t isaac_sim_lab_ros2_nav2:2.1.0 .
    ```

### Option 2: Download Pre-built Container

Available container versions (Isaac Sim 4.5.0):

| Version | Isaac Lab Version | Description |
|---------|-------------------|-------------|
| 1.0.0   | 2.0.0            | Initial release |
| 2.0.0   | 2.1.0            | Updated Isaac Lab |
| 2.1.0   | 2.1.0            | Added git configuration |

## Usage

### Running the Container

Here are examples of how to run the container:

#### Test Run

```bash
docker run --name isaac_sim_lab_container \
    --gpus '"device=0"' \
    -it --rm \
    --network=host \
    --env-file .env \
    isaac_sim_lab_ros2_nav2:2.1.0
```

or

`launch_script.sh` need to change the version you want to launch
```bash
./launch_script.sh 
```

#### Example  (Using User-Specific Paths)

```bash
docker run --name isaac_sim_lab_container \
    --gpus '"device=0"' \
    -it --rm \
    --network=host \
    --env-file .env \
    -v /Path/to/User/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v /Path/to/User/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v /Path/to/User/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v /Path/to/User/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v /Path/to/User/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v /Path/to/User/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v /Path/to/User/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v /Path/to/User/docker/isaac-sim/documents:/root/Documents:rw \
    -v /Path/to/User/docker/Isaac_sim_other_resource:/isaac-sim/Isaac_sim_other_resource:rw \
    -v /Path/to/User/docker/IsaacLab:/opt/IsaacLab/:rw \
    isaac_sim_lab_ros2_nav2:2.1.0
```

### Isaac Lab Git Configuration

* IF version < 2.1.0

```bash
git config --global --add safe.directory /opt/IsaacLab
```


## Testing

### ROS2 and X11

1.  Verify that X11 is correctly configured:

    ```bash
    echo $DISPLAY
    ```

    If the output is not ':1', manually set it:

    ```bash
    export DISPLAY=:1
    ```

2.  Test ROS2:

    ```bash
    ros2 run rviz2 rviz2
    ```

### Isaac Sim and ROS2

1.  Start Isaac Sim:

    ```bash
    ./runheadless.sh
    ```

    Specify the GPU to use
    ```bash
    CUDA_VISIBLE_DEVICES=2 ./isaac-sim.streaming.sh
    ```

2.  Check ROS2 topics:

    ```bash
    ros2 topic list
    ```



## Creating Custom ROS 2 Package in Isaac Sim with Humble Workspace

To create a custom ROS 2 package in Isaac Sim using the `humble_ws` workspace, follow this English version with optimized structure:
### Initial File Structure

```
/opt/humble_ws/src/custom_package/
├── package.xml
├── CMakeLists.txt
└── scripts/
    └── custom_node.py
```
### Edit CMakeLists.txt
Add custom_node.py in install

### Build Commands
```bash
cd /opt/humble_ws
colcon build --packages-select custom_package
source install/setup.bash
```

### Verification
```bash
ros2 pkg list | grep custom_package  # Should output 'custom_package'
```
## Reference
[Isaac Sim Pipeline Documentation](https://hackmd.io/jbBMe9ykQ5-ySglpPR_OZg)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.



