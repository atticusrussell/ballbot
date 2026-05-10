#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/jetson/dofbot_pro_ws/install/setup.bash
gnome-terminal -- bash -c "python3 /home/jetson/dofbot_pro/APP_DOFBOT_PRO/YahboomArm.pyc;exec bash"
gnome-terminal -- bash -c "python3 /home/jetson/dofbot_pro/APP_DOFBOT_PRO/yb-discover.py"
gnome-terminal -- bash -c "python3 /home/jetson/dofbot_pro/APP_DOFBOT_PRO/joystick.py"

