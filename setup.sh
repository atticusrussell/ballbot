#!/bin/bash
set -e

# the following commented out because empty breaks stuff
# vcs import < src/ros2.repos src
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys micro_ros_agent
