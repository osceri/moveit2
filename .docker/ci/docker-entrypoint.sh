#!/bin/bash
set -e


find /opt/ros/$ROS_DISTRO -name tf2* | xargs rm -rf
# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
