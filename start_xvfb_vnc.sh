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