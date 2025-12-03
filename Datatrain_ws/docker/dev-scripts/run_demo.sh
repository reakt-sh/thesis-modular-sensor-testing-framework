#!/usr/bin/env bash

# Config — change if your paths differ
WS_DIR="${WS_DIR:-/ws}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
INSTALL_SETUP="$WS_DIR/install/setup.bash"

# Ensure the Gazebo GUI can show up in your noVNC session
export DISPLAY="${DISPLAY:-:1}"

echo "[run_demo] Sourcing ROS underlay: $ROS_SETUP"
source "$ROS_SETUP"

echo "[run_demo] Building workspace in: $WS_DIR"
colcon build --symlink-install

echo "[run_demo] Sourcing workspace overlay: $INSTALL_SETUP"
source "$INSTALL_SETUP"

echo "ros2 launch rail_demo yourfavorite.launch.py here"
