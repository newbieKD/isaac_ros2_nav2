#!/bin/bash
set -e

# VNC Environment Cleanup Script
# This script cleans up VNC-related files and processes

echo "🧹 VNC Environment Cleanup..."

# Stop any running VNC processes
echo "🛑 Stopping VNC processes..."
pkill -u "$(whoami)" -f "x11vnc" 2>/dev/null || true
pkill -u "$(whoami)" -f "Xvfb" 2>/dev/null || true
pkill -u "$(whoami)" -f "openbox" 2>/dev/null || true

# Clean up PID files
echo "📂 Cleaning up PID files..."
rm -f /home/user/.vnc/xvfb.pid /home/user/.vnc/x11vnc.pid

# Handle .Xauthority issues
XAUTH_FILE="/home/user/.Xauthority"
if [ -e "$XAUTH_FILE" ]; then
    if [ -d "$XAUTH_FILE" ]; then
        echo "🗂️ Removing .Xauthority directory (root-owned)..."
        sudo rm -rf "$XAUTH_FILE" 2>/dev/null || {
            echo "❌ Cannot remove .Xauthority directory"
            echo "Please run manually: sudo rm -rf $XAUTH_FILE"
            echo "This likely happened due to Docker volume mount creating a directory"
        }
    else
        echo "📄 Removing existing .Xauthority file..."
        rm -f "$XAUTH_FILE"
    fi
fi

# Clean up log files (optional)
echo "📋 Cleaning up log files..."
rm -f /home/user/.vnc/logs/*.log 2>/dev/null || true

# Create clean VNC directory structure
echo "📁 Recreating VNC directory structure..."
mkdir -p /home/user/.vnc/logs

echo "✅ VNC environment cleanup completed!"
echo ""
echo "🚀 You can now run: start-xvfb-vnc"
