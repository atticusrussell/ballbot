#!/bin/bash
set -euo pipefail

WORKSPACE_DIR=$(pwd)
IGNORED_SUFFIXES=("_viz" "_gazebo" "_simulation")
IGNORED_DIR=".ignored_pkgs"

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "ERROR: ROS_DISTRO is not set. Source your ROS install first:" >&2
    echo "  source /opt/ros/humble/setup.bash" >&2
    exit 1
fi

if [ ! -d src ]; then
    echo "ERROR: no src/ directory here. Run this from the workspace root." >&2
    exit 1
fi

# ---------------------------------------------------------------- rplidar

echo "=== Installing rplidar driver ==="
sudo apt install -y "ros-$ROS_DISTRO-rplidar-ros"

if [ ! -f /etc/udev/rules.d/rplidar.rules ]; then
    echo "=== Installing rplidar udev rules ==="
    (
        cd /tmp
        wget https://raw.githubusercontent.com/allenh1/rplidar_ros/ros2/scripts/rplidar.rules
        sudo cp rplidar.rules /etc/udev/rules.d/
    )
    sudo udevadm control --reload-rules && sudo udevadm trigger
else
    echo "rplidar.rules already exists. Skipping download and copy."
fi

# ------------------------------------------------- move out sim/viz pkgs

restore_pkgs() {
    [ -d "$IGNORED_DIR" ] || return 0
    echo ""
    echo "=== Restoring simulation/visualization packages ==="
    for suffix in "${IGNORED_SUFFIXES[@]}"; do
        for dir in "$IGNORED_DIR"/*"$suffix"; do
            [ -d "$dir" ] || continue
            pkg=$(basename "$dir")
            echo "Restoring $pkg (marked ignored for future builds)"
            mv "$dir" src/
            # The move above only protects the build inside THIS script. Without a
            # persistent marker the next plain `colcon build` builds these again, and
            # `rosdep install --from-paths src` drags rviz2 / gazebo_ros_pkgs onto the
            # Pi. COLCON_IGNORE covers both: colcon honours it directly, and rosdep
            # honours it via catkin_pkg's DEFAULT_IGNORE_MARKERS. Gitignored, so this
            # is machine-local state.
            touch "src/$pkg/COLCON_IGNORE"
        done
    done
    # rmdir, not rm -rf: if something unexpected is left behind, say so
    rmdir "$IGNORED_DIR" 2>/dev/null || \
        echo "NOTE: $IGNORED_DIR not empty, left in place — check contents."
}

echo "=== Temporarily moving out simulation/visualization packages ==="
mkdir -p "$IGNORED_DIR"

# Restore on ANY exit: success, set -e abort, or Ctrl-C.
trap restore_pkgs EXIT

for dir in src/*; do
    pkg_name=$(basename "$dir")
    # require package.xml — a dir without one is debris, not a package
    if [[ -d "$dir" && -f "$dir/package.xml" ]]; then
        for suffix in "${IGNORED_SUFFIXES[@]}"; do
            if [[ "$pkg_name" == *"$suffix" ]]; then
                echo "Ignoring and moving $pkg_name"
                mv "$dir" "$IGNORED_DIR/"
                break
            fi
        done
    fi
done

# ---------------------------------------------------------------- micro-ROS

echo "=== Cloning micro_ros_setup if needed ==="
if [ ! -d "src/micro_ros_setup" ]; then
    git clone -b "$ROS_DISTRO" https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
else
    echo "src/micro_ros_setup already exists. Skipping clone."
fi

# ---------------------------------------------------------------- deps

echo "=== Installing required build tools ==="
sudo apt install -y python3-vcstool build-essential

echo "=== Running rosdep for hardware-only packages ==="
sudo apt update
rosdep update

# Skipped keys, and why:
#   microxrcedds_agent / micro_ros_agent — built from source below, not apt
rosdep install --from-path src --ignore-src -y \
    --skip-keys microxrcedds_agent \
    --skip-keys micro_ros_agent

# ---------------------------------------------------------------- build

echo "=== Building workspace (hardware-only packages) ==="
colcon build --symlink-install
set +u; source install/setup.bash; set -u

echo "=== Setting up micro-ROS agent (this takes a while) ==="
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
set +u; source install/setup.bash; set -u

# ---------------------------------------------------------------- verify

echo "=== Verifying micro_ros_agent ==="
if ros2 pkg executables micro_ros_agent 2>/dev/null | grep -q micro_ros_agent; then
    echo "OK: micro_ros_agent built."
else
    echo "WARNING: micro_ros_agent not found — bringup will fail." >&2
fi

echo ""
echo "Done. To use the ROS 2 environment in this terminal, run:"
echo "  source install/setup.bash"
echo ""
echo "Then:"
echo "  ros2 launch ballbot_bringup bringup.launch.py"
