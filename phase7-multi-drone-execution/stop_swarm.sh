#!/usr/bin/env bash
# Stop all PX4 SITL instances and Gazebo.
pkill -f "bin/px4" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
echo "Stopped PX4 and Gazebo processes (if any were running)."
