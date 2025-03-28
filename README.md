# Isaac Sim, Isaac Lab, ROS2, and Nav2 Integration

## Introduction

This project integrates Isaac Sim, Isaac Lab, ROS2 (Humble), and Nav2 to facilitate reinforcement learning training and path planning for humanoid robots. It provides a Docker containerized environment for ease of setup and deployment.

## Prerequisites

*   Docker
*   NVIDIA Container Toolkit
*   RealVNC Viewer (optional, for remote GUI access)

## Installation

1.  Navigate to the `isaac_ros2_nav2` directory.
2.  Build the Docker image:

    ```bash
    docker build -t isaac_sim_lab_ros2_nav2 .
    ```

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
    isaac_sim_lab_ros2_nav2:latest
```

#### Example 1 (Mounting Volumes)

```bash
docker run --name isaac_sim_lab_container \
    --gpus '"device=0"' \
    -it --rm \
    --network=host \
    --env-file .env \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/documents:/root/Documents:rw \
    -v /media/Pluto/binghua/Isaac_sim_other_resource:/isaac-sim/Isaac_sim_other_resource:rw \
    -v /media/Pluto/binghua/TSMC_docker/IsaacLab:/opt/IsaacLab/:rw \
    isaac_sim_lab_ros2_nav2:latest
```

#### Example 2 (Using User-Specific Paths)

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
    isaac_sim_lab_ros2_nav2:latest
```

### Isaac Lab Git Configuration

```bash
git config --global --add safe.directory /opt/IsaacLab
```

## VSCode Remote Connection

*   Ensure the VSCode Remote - Containers extension is installed and enabled.
*   **Network Configuration:**
    *   If using `--network=host`, be aware that this can sometimes cause conflicts with VSCode's connection.
    *   Consider using a bridge network instead. If so, create a bridge network: `docker network create my_network`. Then, run the container with `--network my_network`.
    *   When using a bridge network, you'll need to map the necessary ports. For example, add `-p 5900:5900` to your `docker run` command to map the VNC port. VSCode also requires a range of ports for its server.
*   **Firewall:**
    *   Ensure that no firewalls (either on your host machine or network) are blocking the connection to the container on the mapped ports.
*   **User Permissions:**
    *   Verify that the user inside the container has the correct permissions to access the workspace directory (`/opt/IsaacLab` in your case).
*   **VSCode Settings:**
    *   Check your VSCode settings for any configurations that might be interfering with the Remote - Containers extension. Specifically, look for proxy settings if you are behind a proxy.
*   **Docker Context:**
    *   Make sure VSCode is using the correct Docker context.
*   **Rebuild Container:** Sometimes, simply rebuilding the container can resolve connection issues. Try "Remote-Containers: Rebuild Container" in VSCode.

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

2.  Check ROS2 topics:

    ```bash
    ros2 topic list
    ```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


