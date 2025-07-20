#!/bin/bash
set -e

# Stop User-Based X11VNC and Xvfb Services
# This script stops all VNC services running under user privileges

echo "🛑 Stopping User-Based X11VNC Remote Desktop..."

# PID files
XVFB_PIDFILE="/home/user/.vnc/xvfb.pid"
X11VNC_PIDFILE="/home/user/.vnc/x11vnc.pid"

# Function to stop process by PID file
stop_service() {
    local pidfile=$1
    local service_name=$2
    
    if [ -f "$pidfile" ]; then
        local pid=$(cat "$pidfile")
        if kill -0 "$pid" 2>/dev/null; then
            echo "🔄 Stopping $service_name (PID: $pid)..."
            kill "$pid"
            
            # Wait for graceful shutdown
            local count=0
            while kill -0 "$pid" 2>/dev/null && [ $count -lt 10 ]; do
                sleep 1
                count=$((count + 1))
            done
            
            # Force kill if still running
            if kill -0 "$pid" 2>/dev/null; then
                echo "⚠️ Force stopping $service_name..."
                kill -9 "$pid" 2>/dev/null || true
            fi
            
            echo "✅ $service_name stopped"
        else
            echo "ℹ️ $service_name was not running"
        fi
        rm -f "$pidfile"
    else
        echo "ℹ️ $service_name PID file not found"
    fi
}

# Stop X11VNC server
stop_service "$X11VNC_PIDFILE" "X11VNC server"

# Stop Xvfb
stop_service "$XVFB_PIDFILE" "Xvfb display"

# Kill any remaining user-owned VNC processes
echo "🧹 Cleaning up remaining processes..."
pkill -u "$(whoami)" -f "x11vnc" 2>/dev/null || true
pkill -u "$(whoami)" -f "Xvfb" 2>/dev/null || true
pkill -u "$(whoami)" -f "openbox" 2>/dev/null || true

echo ""
echo "✅ All User-Based X11VNC services stopped successfully!"
echo ""
echo "🚀 To start again: /home/user/start_xvfb_vnc.sh"
