#!/bin/bash
set -e

# Install X11VNC and related packages for remote desktop access
# This script installs X11VNC packages and xauth during the Docker build process

echo "Installing X11VNC, Xvfb, xauth and related packages..."

# Install X11VNC, Xvfb, xauth and window managers
apt-get update && apt-get install -y \
    x11vnc \
    xvfb \
    xauth \
    openbox \
    fluxbox \
    xterm \
    tint2 \
    && rm -rf /var/lib/apt/lists/*

echo "X11VNC packages installed successfully!"

# Note: All VNC user configuration will be done in user space after USER switch
