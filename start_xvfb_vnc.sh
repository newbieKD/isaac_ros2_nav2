#!/bin/bash
# 設定 DISPLAY，這裡假設使用 :0
export DISPLAY=:1

# 啟動 Xvfb，如果尚未運行
if ! pgrep Xvfb > /dev/null; then
    Xvfb :1 -screen 0 1920x1080x24 &
    sleep 2
fi

# 啟動 x11vnc，如果尚未運行
if ! pgrep x11vnc > /dev/null; then
    x11vnc -display :1 -forever -nopw -listen 0.0.0.0 -xkb &
fi

# 等待其他應用程序，或直接執行傳入的命令
exec "$@"

# #!/bin/bash
# set -e

# echo "Starting container services..."

# # 清理舊的 X 伺服器鎖定文件
# rm -f /tmp/.X*-lock /tmp/.X11-unix/X*

# # 設定 DISPLAY
# export DISPLAY=:1

# # 確保沒有舊進程
# echo "Stopping any existing services..."
# pkill -9 x11vnc || true
# pkill -9 Xvfb || true
# sleep 2

# # 啟動 Xvfb
# echo "Starting Xvfb..."
# Xvfb :1 -screen 0 1920x1080x24 &
# sleep 2

# # 啟動 x11vnc (使用 -forever 而非 -loop，避免多個實例)
# echo "Starting x11vnc..."
# x11vnc -display :1 -forever -shared -rfbauth /root/.vnc/passwd -rfbport 5900 &
# sleep 2

# # 確認 x11vnc 是否正常啟動
# if pgrep x11vnc > /dev/null; then
#     echo "x11vnc is running successfully"
# else
#     echo "Warning: x11vnc failed to start, retrying..."
#     x11vnc -display :1 -forever -shared -rfbauth /root/.vnc/passwd -rfbport 5900 &
#     sleep 2
# fi

# # 啟動 supervisor 服務
# echo "Starting supervisor service..."
# /usr/bin/supervisord -c /etc/supervisor/supervisord.conf &
# sleep 2

# # 啟動 cron 服務以啟用定時重啟 x11vnc
# echo "Starting cron service for automatic x11vnc restart..."
# service cron start || echo "Failed to start cron service"

# # 設置 ROS2 環境
# source /opt/ros/humble/setup.bash
# source /opt/humble_ws/install/local_setup.bash

# # 等待其他應用程序，或直接執行傳入的命令
# echo "Startup complete, running command: $@"
# exec "$@"

