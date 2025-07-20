#!/bin/bash
set -e

# X11VNC + Xvfb User-Based Startup Script
# This script runs entirely with user privileges (user:user)
# No root permissions required for any VNC operations

echo "🚀 Starting User-Based X11VNC Remote Desktop..."

# Configuration
DISPLAY_NUM=":1"
VNC_PORT="5900"
SCREEN_RESOLUTION="1920x1080x24"
VNC_PASSWORD=0000

# User directories (all under /home/user)
VNC_DIR="/home/user/.vnc"
VNC_LOG_DIR="/home/user/.vnc/logs"
XAUTH_FILE="/home/user/.Xauthority"
VNC_PASSWD_FILE="/home/user/.vnc/passwd"
XVFB_PIDFILE="/home/user/.vnc/xvfb.pid"
X11VNC_PIDFILE="/home/user/.vnc/x11vnc.pid"

# Ensure directories exist
mkdir -p "$VNC_LOG_DIR"

echo "📋 Configuration:"
echo "   Display: $DISPLAY_NUM"
echo "   Port: $VNC_PORT"
echo "   Resolution: $SCREEN_RESOLUTION"
echo "   VNC Password: $VNC_PASSWORD"
echo "   Xauth File: $XAUTH_FILE"
echo "   VNC Password File: $VNC_PASSWD_FILE"

# Function to check if process is running
is_process_running() {
    local pidfile=$1
    if [ -f "$pidfile" ]; then
        local pid=$(cat "$pidfile")
        if kill -0 "$pid" 2>/dev/null; then
            return 0
        else
            rm -f "$pidfile"
        fi
    fi
    return 1
}

# Stop any existing services
echo "🧹 Cleaning up any existing VNC services..."
pkill -u "$(whoami)" -f "Xvfb.*$DISPLAY_NUM" 2>/dev/null || true
pkill -u "$(whoami)" -f "x11vnc.*$VNC_PORT" 2>/dev/null || true
sleep 1

# Create VNC password file (user-owned)
echo "🔐 Creating VNC password file..."
# echo "$VNC_PASSWORD" | vncpasswd -f > "$VNC_PASSWD_FILE" 2>/dev/null || {
#     # Fallback: create password file manually
#     echo "Using fallback VNC password creation..."
#     printf '\x17\x52\x6b\x06\x23\x4e\x58\x07' > "$VNC_PASSWD_FILE"
# }
# chmod 600 "$VNC_PASSWD_FILE"
echo $VNC_PASSWORD | x11vnc -storepasswd $VNC_PASSWORD "$VNC_PASSWD_FILE"
chmod 600 "$VNC_PASSWD_FILE"


# Start Xvfb (X Virtual Framebuffer) as user
echo "🖥️ Starting Xvfb virtual display..."
Xvfb "$DISPLAY_NUM" \
    -screen 0 "$SCREEN_RESOLUTION" \
    -nolisten tcp \
    -nolisten unix \
    -ac \
    -pn \
    -noreset \
    +extension GLX \
    +render \
    -dpi 96 \
    > "$VNC_LOG_DIR/xvfb.log" 2>&1 &

XVFB_PID=$!
echo $XVFB_PID > "$XVFB_PIDFILE"
echo "✅ Xvfb started with PID: $XVFB_PID"

# Wait for Xvfb to be ready
sleep 3

# Set environment variables for this display
export DISPLAY="$DISPLAY_NUM"
export XAUTHORITY="$XAUTH_FILE"

# Remove any existing Xauthority file/directory and create new one
echo "🔑 Preparing Xauthority file..."
if [ -e "$XAUTH_FILE" ]; then
    if [ -d "$XAUTH_FILE" ]; then
        echo "⚠️ Xauthority is a directory, attempting to remove..."
        sudo rm -rf "$XAUTH_FILE" 2>/dev/null || {
            echo "❌ Cannot remove Xauthority directory (root-owned)"
            echo "Please run: sudo rm -rf $XAUTH_FILE"
            exit 1
        }
    else
        rm -f "$XAUTH_FILE"
    fi
fi

# Create empty Xauthority file with correct ownership
touch "$XAUTH_FILE"
chmod 600 "$XAUTH_FILE"

# Generate Xauthority for the display (user-owned)
echo "🔑 Generating Xauthority file..."
xauth generate "$DISPLAY_NUM" . trusted 2>/dev/null || {
    echo "⚠️ xauth generate failed, creating manual entry..."
    # Create xauth entry manually
    xauth add "$DISPLAY_NUM" . $(openssl rand -hex 16) 2>/dev/null || true
}

# Verify Xauthority file exists and is owned by user
if [ ! -f "$XAUTH_FILE" ]; then
    echo "⚠️ Creating minimal Xauthority file..."
    touch "$XAUTH_FILE"
    chmod 600 "$XAUTH_FILE"
fi

echo "📁 Xauthority file created: $XAUTH_FILE"

# Start window manager (optional but recommended)
echo "🪟 Starting lightweight window manager..."
# openbox --config-file /dev/null > "$VNC_LOG_DIR/openbox.log" 2>&1 &
openbox-session > "$VNC_LOG_DIR/openbox.log" 2>&1 &

# Wait for window manager to initialize
sleep 2

# Start tint2 panel
tint2 > "$VNC_LOG_DIR/tint2.log" 2>&1 &

# Start X11VNC server with explicit user-owned xauth
echo "🌐 Starting X11VNC server..."
x11vnc \
    -display "$DISPLAY_NUM" \
    -rfbport "$VNC_PORT" \
    -rfbauth "$VNC_PASSWD_FILE" \
    -xauth "$XAUTH_FILE" \
    -shared \
    -forever \
    -noxdamage \
    -noxfixes \
    -noxrandr \
    -wait 10 \
    -defer 10 \
    -speeds dsl \
    -readtimeout 20 \
    -nap \
    -cursor arrow \
    -arrow 3 \
    -nosel \
    -noprimary \
    -noclipboard \
    -nosetclipboard \
    -logappend \
    -quiet \
    -bg \
    -o "$VNC_LOG_DIR/x11vnc.log"

# Wait for X11VNC to start
sleep 2

# Get and save X11VNC PID
X11VNC_PID=$(pgrep -u "$(whoami)" -f "x11vnc.*$VNC_PORT" | head -1)
if [ -n "$X11VNC_PID" ]; then
    echo $X11VNC_PID > "$X11VNC_PIDFILE"
    echo "✅ X11VNC server started with PID: $X11VNC_PID"
else
    echo "❌ Failed to start X11VNC server"
    exit 1
fi

# Verify all services are running
echo ""
echo "🔍 Verifying services..."
if is_process_running "$XVFB_PIDFILE"; then
    echo "✅ Xvfb is running (PID: $(cat $XVFB_PIDFILE))"
else
    echo "❌ Xvfb is not running"
    exit 1
fi

if is_process_running "$X11VNC_PIDFILE"; then
    echo "✅ X11VNC is running (PID: $(cat $X11VNC_PIDFILE))"
else
    echo "❌ X11VNC is not running"
    exit 1
fi

echo ""
echo "🎉 User-Based X11VNC Remote Desktop is ready!"
echo ""
echo "📋 Connection Information:"
echo "   🌐 VNC Address: localhost:$VNC_PORT"
echo "   🔑 VNC Password: $VNC_PASSWORD"
echo "   📺 Display: $DISPLAY_NUM"
echo "   🖥️ Resolution: ${SCREEN_RESOLUTION//x/ x }"
echo ""
echo "🔧 Environment Variables (for GUI applications):"
echo "   export DISPLAY=$DISPLAY_NUM"
echo "   export XAUTHORITY=$XAUTH_FILE"
echo ""
echo "📱 Connect with any VNC client:"
echo "   • Desktop: vncviewer localhost:$VNC_PORT"
echo "   • Network: Use docker-host-ip:$VNC_PORT"
echo ""
echo "🧪 Test with GUI applications:"
echo "   xterm &"
echo "   openbox-session &"
echo "   # Then run Isaac Sim or ROS2 GUI tools"
echo ""
echo "📊 Monitor services:"
echo "   ps aux | grep -E '(Xvfb|x11vnc)' | grep \$(whoami)"
echo "   tail -f $VNC_LOG_DIR/x11vnc.log"
echo ""
echo "🛑 Stop services: /home/user/stop_xvfb_vnc.sh"
