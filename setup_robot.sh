#!/bin/bash
set -e

WORKSPACE_DIR=$(pwd)

# install rplidar ros2 drivers
sudo apt install -y ros-$ROS_DISTRO-rplidar-ros
# Check if rplidar.rules already exists in /etc/udev/rules.d/
if [ ! -f /etc/udev/rules.d/rplidar.rules ]; then
    cd /tmp
    wget https://raw.githubusercontent.com/allenh1/rplidar_ros/ros2/scripts/rplidar.rules
    sudo cp rplidar.rules /etc/udev/rules.d/
    cd "$WORKSPACE_DIR"
else
    echo "rplidar.rules already exists. Skipping download and copy."
fi

# Make colcon ignore non-robot packages (simulation and X11 dependencies)
touch src/brobot_gazebo/COLCON_IGNORE
touch src/brobot_viz/COLCON_IGNORE
touch src/catbot_simulation/COLCON_IGNORE

# hide all brobot packages so they're not seen by rosdep
for dir in brobot* catbot*; do
    if [ -d "src/$dir" ]; then
        mv "src/$dir" ../..
    fi
done

# Download and install micro-ROS
cd "$WORKSPACE_DIR"

if [ ! -d "src/micro_ros_setup" ]; then
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
else
    echo "src/micro_ros_setup already exists. Skipping clone."
fi

sudo apt install python3-vcstool build-essential
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
colcon build
source install/setup.bash

# Setup micro-ROS agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash

# Move brobot packages back
for dir in brobot* catbot*; do
    if [ -d "../../$dir" ]; then
        mv "../../$dir" src/
    fi
done

# Install brobot packages
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent --skip-keys micro_ros_agent
colcon build
source install/setup.bash
