#!/bin/bash
 
set -e

# Ros build
source "/opt/ros/noetic/setup.bash"

echo "==============Direct Lidar Odometry Noetic Docker Env Ready================"

cd /root/workspace

exec "$@"