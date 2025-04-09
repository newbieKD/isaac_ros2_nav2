# 使用 Isaac Sim 官方容器為基底
FROM nvcr.io/nvidia/isaac-sim:4.5.0 AS base

# 設定環境變數以避免互動式提示
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Taipei

# 設定 TERM 環境變數
ENV TERM=xterm

# 設定X11顯示變數
ENV DISPLAY=:1

# 更新並安裝必要工具與依賴
RUN apt-get update && apt-get install -y --no-install-recommends \
    htop \
    iotop \
    net-tools \
    supervisor \
    tmux \
    locales \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    git \
    build-essential \
    cmake \
    x11vnc \
    xvfb \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------
# 安裝Isaac Lab
# -----------------------------------------------------
FROM base AS isaac_lab_layer
# 切換工作目錄到 /opt，這裡用來存放 Isaac Lab 代碼
WORKDIR /opt

# 從官方 GitHub 倉庫克隆 Isaac Lab
RUN git clone https://github.com/isaac-sim/IsaacLab.git

# 切換到 IsaacLab 目錄
WORKDIR /opt/IsaacLab

# 建立符號連結，使得 Isaac Lab 能夠正確索引 Isaac Sim 的 Python 模組與擴展
# 假設 Isaac Sim 安裝在容器內的 /isaac-sim 目錄下
RUN ln -s /isaac-sim _isaac_sim
RUN apt install cmake build-essential

# 執行 Isaac Lab 的安裝腳本，安裝必要依賴並編譯 Isaac Lab
RUN ./isaaclab.sh --install

# -----------------------------------------------------
# 安裝 ROS2 Humble 與 Nav2
# -----------------------------------------------------
FROM isaac_lab_layer AS ros2_original_layer
# 注意：ROS2 Humble 官方支援 Ubuntu 22.04 (jammy)
# 即使 host 為 Ubuntu 20.04，容器內部環境是獨立的，
# 因此我們可以使用 jammy 的 ROS2 repository 安裝 Humble
ENV ROS_DISTRO=humble

# 加入 ROS2 GPG 金鑰與 repository（指定 jammy 來源）
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update

# 安裝 ROS2 Humble 的依賴
RUN apt-get update && apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6

# 安裝 ROS2 Humble desktop 與 Nav2 套件
RUN apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-vision-msgs \
    ros-humble-ackermann-msgs

# 設定 ROS2 環境變數
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

# -----------------------------------------------------
# 設定 ROS2 工作區
# -----------------------------------------------------
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository universe && apt-get update

RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

RUN apt-get update && apt-get install -y python3-colcon-common-extensions

# ------------------------------------------------------
# update to isaac sim & ROS2 tutorials workspaces
# ------------------------------------------------------
FROM ros2_original_layer AS ros2_isaac_layer
# Clone the ROS2 workspace for Isaac Sim
WORKDIR /opt
RUN git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git

# Move the workspace to /opt
RUN mv IsaacSim-ros_workspaces/humble_ws /opt/

# Remove the other file
RUN rm -rf /opt/IsaacSim-ros_workspaces

# Create a workspace for original ROS2 workspaces and download dependencies
RUN rosdep init && \
    rosdep update || echo "rosdep update failed, check network"
RUN bash -c "source /opt/ros/humble/setup.bash && \
        cd /opt/humble_ws && \
        rosdep install -i --from-path src --rosdistro humble -y && \
        colcon build"

# 設定 ROS2 工作區環境變數
RUN echo "source /opt/humble_ws/install/local_setup.bash" >> /root/.bashrc

# -----------------------------------------------------
# 設置 Isaac Sim 與 ROS2 橋接
# -----------------------------------------------------
RUN pip install roslibpy
RUN echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc
RUN echo "export ROS_IP=127.0.0.1" >> /root/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc
RUN echo "export ISAACSIM_ROS2_BRIDGE=1" >> /root/.bashrc

# -----------------------------------------------------
# 設定啟動 Xvfb 與 VNC 的腳本（用於 headless 的 GUI 轉發）
# -----------------------------------------------------
FROM ros2_isaac_layer AS final

# VNC 設定
RUN mkdir -p /var/log/x11vnc \
    && mkdir -p /root/.vnc \
    && x11vnc -storepasswd 0000 /root/.vnc/passwd

# 時區與語系設定
RUN ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    echo $TZ > /etc/timezone && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 複製配置文件
COPY x11vnc.conf /etc/supervisor/conf.d/
COPY init_services.sh /opt/startup/init_services.sh
RUN chmod +x /opt/startup/init_services.sh

# -----------------------------------------------------
# 安裝 Node.js 和 npm for github copilot
# -----------------------------------------------------
# 使用 NodeSource 官方腳本安裝最新的 LTS 版本
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------
# 默認工作目錄與啟動命令
# -----------------------------------------------------
WORKDIR /opt/IsaacLab

ENTRYPOINT ["/bin/bash", "-c", "/opt/startup/init_services.sh && tail -f /dev/null"]