#!/bin/bash
set -e

# Install Isaac Lab

# Check required environment variables
if [ -z "$USERNAME" ]; then
    echo "Error: USERNAME environment variable is required but not set"
    exit 1
fi
if [ -z "$TARGETARCH" ]; then
    echo "Error: TARGETARCH environment variable is required but not set"
    exit 1
fi
if [ -z "$ISAAC_SIM_VERSION" ]; then
    echo "Error: ISAAC_SIM_VERSION environment variable is required but not set"
    exit 1
fi
if [ -z "$ISAACSIM_PATH" ]; then
    echo "Error: ISAACSIM_PATH environment variable is required but not set"
    exit 1
fi
if [ -z "$ISAAC_LAB_VERSION" ]; then
    echo "Skipping Isaac Lab installation as ISAAC_LAB_VERSION is not set"
    exit 0
fi
if [ -z "$ISAACLAB_PATH" ]; then
    echo "Error: ISAACLAB_PATH environment variable is required but not set"
    exit 1
fi

echo "Installing Isaac Lab for architecture: $TARGETARCH"
# echo "Isaac Lab version: $ISAAC_LAB_VERSION"

# Only install Isaac Lab on amd64 architecture
if [ "$TARGETARCH" != "amd64" ]; then
    echo "Skipping Isaac Lab installation for architecture: $TARGETARCH (only supported on amd64)"
    exit 0
fi

if [ "$ISAAC_SIM_VERSION" = "4.5.0" ]; then
    ISAAC_LAB_VERSION="2.1.0"
elif [ "$ISAAC_SIM_VERSION" = "5.0.0" ]; then
    ISAAC_LAB_VERSION="2.2.0"
else
    echo "Error: Unsupported Isaac Sim version: $ISAAC_LAB_VERSION"
    exit 1
fi

echo "Installing Isaac Lab $ISAAC_LAB_VERSION..."
sudo apt-get update && sudo apt-get install -y \
    cmake build-essential \
    && sudo rm -rf /var/lib/apt/lists/* \
    || exit 1
git clone --branch "v$ISAAC_LAB_VERSION" https://github.com/isaac-sim/IsaacLab.git "$ISAACLAB_PATH" \
    && cd "$ISAACLAB_PATH" \
    && ln -s "$ISAACSIM_PATH" _isaac_sim \
    && bash ./isaaclab.sh --install \
    || exit 1
echo "Isaac Lab installation completed successfully!"
