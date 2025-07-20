# Setup paths in `~/.profile` to allow unified environment variable across login/non-login shells
# set PATH so it includes user's private bin if it exists
if [ -d "$HOME/bin" ] ; then
    PATH="$HOME/bin:$PATH"
fi
# set PATH so it includes user's private bin if it exists
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi

# Source global ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
# Source colcon-argcomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
# Optionally perform apt update if it has not been executed yet
# if [ -z "$( ls -A '/var/lib/apt/lists' )" ]; then
#     echo "apt-get update has not been executed yet. Running sudo apt-get update..."
#     sudo apt-get update
# fi
# Optionally perform rosdep update if it has not been executed yet
if [ ! -d $HOME/.ros/rosdep/sources.cache ]; then
    echo "rosdep update has not been executed yet. Running rosdep update..."
    rosdep update --rosdistro $ROS_DISTRO
    cd $ROS2_WS
    # Ref: https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
    rosdep install --from-paths src --ignore-src -y -r
fi
# Optionally build the workspace if it has not been built yet
# if [ ! -f $ROS2_WS/install/setup.bash ]; then
#     echo "Workspace has not been built yet. Building workspace..."
#     cd $ROS2_WS
#     # TODO: If command `arch` outputs `aarch64`, consider adding `--packages-ignore <package>` to ignore x86 packages
#     # Ref: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
#     if [ $(arch) == "aarch64" ]; then
#         colcon build --symlink-install
#     else
#         colcon build --symlink-install
#     fi
#     echo "Workspace built."
# fi

if [ ! -f $ROS2_WS/install/setup.bash ] && [ -d $ROS2_WS/src ]; then
    echo "Workspace has not been built yet. Building workspace..."
    cd $ROS2_WS
    colcon build --symlink-install
    echo "Workspace built."
elif [ ! -d $ROS2_WS/src ]; then
    echo "WARNING: $ROS2_WS/src not found, skipping workspace build."
fi


# Source gazebo environment
# Ref: https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros#InstallGazebo
if [ $(arch) == "x86_64" ]; then
  source /usr/share/gazebo/setup.bash
fi
# TODO: Source other workspace environments as underlay
# Source workspace environment
source $ROS2_WS/install/setup.bash
echo "Successfully built workspace and configured environment variables."

# User-Based X11VNC Remote Desktop Aliases and Functions
alias start-xvfb-vnc='/home/user/start_xvfb_vnc.sh'
alias stop-xvfb-vnc='/home/user/stop_xvfb_vnc.sh'
alias cleanup-vnc='/home/user/cleanup_vnc.sh'
alias vnc-user-status='ps aux | grep -E "(Xvfb|x11vnc)" | grep $(whoami) | grep -v grep'
alias vnc-user-logs='tail -f /home/user/.vnc/logs/x11vnc.log'

# Function to set display for VNC usage
vnc-display() {
    export DISPLAY=":1"
    export XAUTHORITY="/home/user/.Xauthority"
    echo "Display set to :1 for User-Based VNC usage"
    echo "XAUTHORITY set to /home/user/.Xauthority"
    echo "You can now run GUI applications that will appear in VNC"
}

# Function to set display for X11 forwarding
x11-display() {
    export DISPLAY="$DISPLAY_ORIG"
    unset XAUTHORITY
    echo "Display restored to $DISPLAY_ORIG for X11 forwarding"
}

# Auto-switch DISPLAY/XAUTHORITY to VNC if not using X11 forwarding (DISPLAY does not start with 'localhost')
if [[ "$DISPLAY" != localhost* && -e /home/user/.Xauthority ]]; then
    vnc-display
fi



# Save original DISPLAY variable
export DISPLAY_ORIG="$DISPLAY"

# If not using X11 forwarding (DISPLAY does not start with 'localhost')
# and user-owned Xauthority file exists, automatically set up VNC environment.
if [[ "$DISPLAY" != localhost* && -e /home/user/.Xauthority ]]; then
    vnc-display
fi


echo ""
echo "🖥️ User-Based X11VNC Remote Desktop commands available:"
echo "   start-xvfb-vnc  - Start user-owned Xvfb + X11VNC services"
echo "   stop-xvfb-vnc   - Stop user-owned X11VNC services"
echo "   cleanup-vnc     - Clean up VNC environment (fix issues)"
echo "   vnc-user-status - Check user VNC processes"
echo "   vnc-user-logs   - View user VNC logs"
echo "   vnc-display     - Set DISPLAY for user VNC (:1)"
echo "   x11-display     - Set DISPLAY for X11 forwarding"
