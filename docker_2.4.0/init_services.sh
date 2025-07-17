#!/bin/bash

# 設置 ROS2 環境
source /opt/ros/humble/setup.bash
source /opt/humble_ws/install/local_setup.bash

# 啟動 supervisor 來管理 x11vnc (在背景運行，使用 sudo 因為是非 root 用戶)
sudo /usr/bin/supervisord -c /etc/supervisor/supervisord.conf &

# 輸出初始化完成訊息
echo "Custom VNC services initialized in background"

