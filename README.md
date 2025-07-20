# Isaac Sim, Isaac Lab, ROS2, and Nav2 Integration

## Introduction

This project integrates Isaac Sim, Isaac Lab, ROS2 (Humble), and Nav2 to facilitate reinforcement learning training and path planning for humanoid robots. It provides a Docker containerized environment for ease of setup and deployment.

## Prerequisites

*   Docker
*   NVIDIA Container Toolkit
*   RealVNC Viewer (optional, for remote GUI access)

## Installation

### Option 1: Build Docker Image

1.  Navigate to the `isaac_ros2_nav2/corporate_docker/docker` directory.
2.  Build the Docker image:

    ```bash
    docker compose build
    ```

### Option 2: Download Pre-built Container

Available container versions (Isaac Sim 4.5.0):

| Version | Isaac Lab Version | Description |
|---------|-------------------|-------------|
| 1.0.0   | 2.0.0            | Initial release |
| 2.0.0   | 2.1.0            | Updated Isaac Lab |
| 2.1.0   | 2.1.0            | Added git configuration |
| 2.2.0   | 2.1.0            | Added none-root user |
| 2.4.0   | 2.1.0            | Fix issue |
| 3.0.0   | 2.1.0            | module_docker |

## Version-Specific Setup Instructions

Each version has detailed setup and usage instructions in their respective subdirectories:

### Available Version Documentation

- **Version 1.0.0**: See [docker_1.0.0/README.md](docker_1.0.0/README.md) for setup and usage instructions
- **Version 2.4.0**: See [docker_2.4.0/README.md](docker_2.4.0/README.md) for setup and usage instructions  
- **Corporate/Enterprise Version**: See [corporate_docker/README.md](corporate_docker/README.md) for enterprise-grade setup with enhanced security features


**Note**: Each subdirectory contains version-specific Docker configurations, environment files, and detailed usage examples. Please refer to the appropriate README.md file based on your target version.




## Reference
[Isaac Sim Pipeline Documentation](https://hackmd.io/jbBMe9ykQ5-ySglpPR_OZg)


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.



