# Base Image: https://hub.docker.com/_/ubuntu/tags?name=22.04
FROM ubuntu:22.04 AS amd64
# Base Image: https://hub.docker.com/_/ubuntu/tags?name=22.04
FROM ubuntu:22.04 AS arm64

# Use docker automatic platform args to select the base image.
# It may be `arm64` or `amd64` depending on the platform.
# Ref: https://docs.docker.com/reference/dockerfile/#automatic-platform-args-in-the-global-scope
FROM $TARGETARCH
ARG TARGETARCH

# Keep downloaded packages for caching purposes
# Ref: https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#example-cache-apt-packages
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Upgrade packages
# Ref: https://pythonspeed.com/articles/security-updates-in-docker/
# Ref: https://github.com/moby/buildkit/blob/master/frontend/dockerfile/docs/reference.md#example-cache-apt-packages
# Ref: https://github.com/moby/buildkit/issues/1673#issuecomment-1264502398
# Ref: https://github.com/moby/buildkit/issues/1673#issuecomment-1987107404
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get upgrade -y \
    && rm -rf /var/lib/apt/lists/*

# Install common tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    curl \
    git \
    git-lfs \
    htop \
    iputils-ping \
    jq \
    nano \
    net-tools \
    netcat-traditional \
    nmap \
    tmux \
    tree \
    unzip \
    vim \
    wget \
    zip \
    p7zip-full \
    && rm -rf /var/lib/apt/lists/*

# Install Python pip
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ROS version configuration
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=$ROS_DISTRO
# Copy and run ROS installation script
COPY modules/install_ros.sh /tmp/install_ros.sh
RUN chmod +x /tmp/install_ros.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_ros.sh && rm /tmp/install_ros.sh \
    && chown -R $USERNAME:$USERNAME /opt/ros/$ROS_DISTRO
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    libxkbcommon-x11-0 libxcb-xinerama0 libxshmfence1 \
    libglu1-mesa libgl1-mesa-dri mesa-utils x11-apps \
    && rm -rf /var/lib/apt/lists/*


# Set the CycloneDDS configuration file
ENV CYCLONEDDS_URI=/home/user/cyclonedds.xml

# Setup the required capabilities for the container runtime
# Ref: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html#driver-capabilities
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# Install GUI debugging tools and configure Vulkan
COPY modules/install_x11_opengl_vulkan.sh /tmp/install_x11_opengl_vulkan.sh
RUN chmod +x /tmp/install_x11_opengl_vulkan.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_x11_opengl_vulkan.sh && rm /tmp/install_x11_opengl_vulkan.sh

# Install X11VNC and Xvfb for remote desktop access
COPY modules/install_vnc.sh /tmp/install_vnc.sh
RUN chmod +x /tmp/install_vnc.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_vnc.sh && rm /tmp/install_vnc.sh

# Install CUDA Toolkit (not installed by default)
ARG CUDA_TOOLKIT_VERSION=""
# Copy and run CUDA Toolkit installation script
COPY modules/install_cuda_toolkit.sh /tmp/install_cuda_toolkit.sh
RUN chmod +x /tmp/install_cuda_toolkit.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_cuda_toolkit.sh && rm /tmp/install_cuda_toolkit.sh
# Ref: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/#post-installation-actions
ENV PATH="${PATH}:/usr/local/cuda-${CUDA_TOOLKIT_VERSION}/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/usr/local/cuda-${CUDA_TOOLKIT_VERSION}/lib64"

# Arguments for the default user
ARG USERNAME=user
ARG USER_UID=1000

# Install sudo and create a user with sudo privileges
# Ref: https://stackoverflow.com/a/65434659
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash -u $USER_UID -G sudo $USERNAME \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/* \
    && echo "$USERNAME:0000" | chpasswd

USER $USERNAME

# Create cache directories with correct ownership to avoid permission issues after volume mount
RUN mkdir -p /home/$USERNAME/.cache/pip
# Create Gazebo cache directory with correct ownership to avoid permission issues after volume mount
RUN mkdir /home/$USERNAME/.gazebo

# Create VNC and scripts directories for user-owned VNC operations
RUN mkdir -p /home/$USERNAME/.vnc \
    && mkdir -p /home/$USERNAME/.vnc/logs

USER root
# Isaac Sim version configuration
# ARG ISAAC_SIM_VERSION=4.5.0
ARG ISAAC_SIM_VERSION=5.0.0
# Ref: https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/binaries_installation.html
ENV ISAACSIM_PATH="/home/$USERNAME/isaacsim"
# Copy and run Isaac Sim installation script
COPY --chown=$USERNAME:$USERNAME \
     modules/install_isaac_sim.sh /tmp/install_isaac_sim.sh
RUN chmod +x /tmp/install_isaac_sim.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private,uid=$USER_UID,gid=$USER_UID \
    /tmp/install_isaac_sim.sh && rm /tmp/install_isaac_sim.sh



# Set Isaac Sim environment variables
# Ref: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#running-isaac-sim
# Ref: https://github.com/NVIDIA-Omniverse/IsaacSim-dockerfiles/blob/e3c09375c2d110b39c3fab3611352870aa3ce6ee/Dockerfile.2023.1.0-ubuntu22.04#L49-L53
ENV OMNI_USER=admin
ENV OMNI_PASS=admin
ENV OMNI_KIT_ACCEPT_EULA=YES
# Ref: https://isaac-sim.github.io/IsaacLab/v2.1.0/source/setup/installation/binaries_installation.html
ENV ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

# Set TERM to prevent Isaac Lab error during docker build:
#    'ansi+tabs': unknown terminal type.
ENV TERM=xterm-256color
# Isaac Lab version configuration
# 根據 ISAAC_LAB_VERSION 的版本改變 (in install_isaac_lab.sh)
ARG ISAAC_LAB_VERSION=2.1.0 
# Ref: https://isaac-sim.github.io/IsaacLab/v2.1.0/source/policy_deployment/00_hover/hover_policy.html
ENV ISAACLAB_PATH="/home/$USERNAME/IsaacLab"
COPY --chown=$USERNAME:$USERNAME \
     modules/install_isaac_lab.sh /tmp/install_isaac_lab.sh
RUN chmod +x /tmp/install_isaac_lab.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    --mount=type=cache,target=/home/$USERNAME/.cache/pip,sharing=private,uid=$USER_UID,gid=$USER_UID \
    /tmp/install_isaac_lab.sh && rm /tmp/install_isaac_lab.sh



# Isaac ROS version configuration
ARG ISAAC_ROS=YES
ENV ISAAC_ROS_WS="/home/$USERNAME/workspaces/isaac_ros-dev/"
# Copy and run Isaac ROS installation script
COPY --chown=$USERNAME:$USERNAME \
     modules/install_isaac_ros.sh /tmp/install_isaac_ros.sh
RUN chmod +x /tmp/install_isaac_ros.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    /tmp/install_isaac_ros.sh && rm /tmp/install_isaac_ros.sh
# Ref: https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html


# Install custom tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update && sudo apt-get install -y \
    git-extras \
    && sudo rm -rf /var/lib/apt/lists/*

# TODO: Add more commands here


COPY --chown=$USERNAME:$USERNAME \
     .bashrc /home/$USERNAME/.bashrc
# Copy User-Based VNC scripts from modules
COPY --chown=$USERNAME:$USERNAME \
     modules/start_xvfb_vnc.sh /home/$USERNAME/start_xvfb_vnc.sh
RUN chmod +x /home/$USERNAME/start_xvfb_vnc.sh
COPY --chown=$USERNAME:$USERNAME \
     modules/stop_xvfb_vnc.sh /home/$USERNAME/stop_xvfb_vnc.sh
RUN chmod +x /home/$USERNAME/stop_xvfb_vnc.sh
COPY --chown=$USERNAME:$USERNAME \
     modules/cleanup_vnc.sh /home/$USERNAME/cleanup_vnc.sh
RUN chmod +x /home/$USERNAME/cleanup_vnc.sh
# Set execute permissions for VNC scripts
RUN chmod +x /home/$USERNAME/start_xvfb_vnc.sh \
    && chmod +x /home/$USERNAME/stop_xvfb_vnc.sh \
    && chmod +x /home/$USERNAME/cleanup_vnc.sh
# TODO: Copy additional files here

# After installing Isaac Sim, force reinstall numpy to ensure compatibility
RUN $ISAACSIM_PYTHON_EXE -m pip install --force-reinstall --no-deps "numpy==1.26.0"

# remove NVIDIA ICD file to prevent Vulkan errors
RUN rm -f /etc/vulkan/icd.d/nvidia_icd.json || true \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME \
    && mkdir -p /home/ros2-essentials \
    && chown -R $USERNAME:$USERNAME /home/ros2-essentials 
    # && deluser $USERNAME sudo



# RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/IsaacLab
# RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/workspaces/isaac_ros-dev/
# RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/isaacsim
# RUN chown -R $USERNAME:$USERNAME /home/$USERNAME

# RUN mkdir -p /home/ros2-essentials \
#     && chown -R $USERNAME:$USERNAME /home/ros2-essentials
# Remove sudo privileges from the user for security
# RUN deluser $USERNAME sudo

# === Prepare Isaac Sim Asset Browser cache structure for universal use ===
# 建立 OV cache 子目錄 & Asset Browser cache
RUN mkdir -p /home/$USERNAME/.cache/ov/isaacsim_asset_browser \
    && mkdir -p /home/$USERNAME/isaacsim/exts/isaacsim.asset.browser/cache \
    && [ -s /home/$USERNAME/isaacsim/exts/isaacsim.asset.browser/cache/isaacsim.asset.browser.cache.json ] \
       || echo "{}" > /home/$USERNAME/isaacsim/exts/isaacsim.asset.browser/cache/isaacsim.asset.browser.cache.json \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/.cache /home/$USERNAME/isaacsim
# 可選：設定 Kit 全域 cache_root
RUN mkdir -p /home/$USERNAME/.nvidia-omniverse/config \
    && printf "[paths]\ncache_root = \"%s\"\n" "/home/$USERNAME/.cache/ov" \
    > /home/$USERNAME/.nvidia-omniverse/config/omniverse.toml \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/.nvidia-omniverse

USER $USERNAME

RUN mkdir -p /home/ros2-essentials/humble_ws

ENTRYPOINT []
# Default: Start bash shell
# Optional: Uncomment to auto-start user-based X11VNC on container startup
# CMD ["/home/user/start_xvfb_vnc.sh", "&&", "/bin/bash"]
CMD ["/bin/bash"]
