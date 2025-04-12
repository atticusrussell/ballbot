#!/bin/bash
set -e

WORKSPACE_DIR=$(pwd)
IGNORED_SUFFIXES=("_viz" "_gazebo" "_simulation")
IGNORED_DIR=".ignored_pkgs"

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

echo "=== Temporarily moving out simulation/visualization packages ==="
mkdir -p "$IGNORED_DIR"

for dir in src/*; do
    pkg_name=$(basename "$dir")
    if [[ -d "$dir" ]]; then
        for suffix in "${IGNORED_SUFFIXES[@]}"; do
            if [[ "$pkg_name" == *"$suffix" ]]; then
                echo "Ignoring and moving $pkg_name"
                touch "$dir/COLCON_IGNORE"
                mv "$dir" "$IGNORED_DIR/"
                break
            fi
        done
    fi
done

# Download and install micro-ROS
echo "=== Cloning micro_ros_setup if needed ==="
if [ ! -d "src/micro_ros_setup" ]; then
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
else
    echo "src/micro_ros_setup already exists. Skipping clone."
fi

echo "=== Installing required build tools ==="
sudo apt install python3-vcstool build-essential

echo "=== Running rosdep for hardware-only packages ==="
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y \
    --skip-keys microxrcedds_agent \
    --skip-keys micro_ros_agent

echo "=== Building workspace (hardware-only packages) ==="
colcon build
source install/setup.bash

echo "=== Setting up micro-ROS agent ==="
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash

echo "=== Restoring simulation/visualization packages ==="
for suffix in "${IGNORED_SUFFIXES[@]}"; do
    for dir in "$IGNORED_DIR"/*"$suffix"; do
        [ -d "$dir" ] || continue
        echo "Restoring $(basename "$dir")"
        mv "$dir" src/
    done
done

echo ""
echo "To use the ROS 2 environment in this terminal, run:"
echo "  source install/setup.bash"
