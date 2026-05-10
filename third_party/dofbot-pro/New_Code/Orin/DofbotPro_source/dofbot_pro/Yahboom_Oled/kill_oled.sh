#!/bin/bash

SCRIPT_PATH="/home/jetson/dofbot_pro/Yahboom_Oled/yahboom_oled.py"

len1=$(ps -ef | grep yahboom_oled.py | grep -v grep | wc -l)
echo "Number of processes=$len1"

if [ $len1 -eq 0 ]; then
    echo "yahboom_oled.py is not running"
else
    oled_pid=$(ps -ef | grep yahboom_oled.py | grep -v grep | awk '{print $2}')
    kill -9 $oled_pid
    echo "yahboom_oled.py killed, PID: $oled_pid"
    
    if python3 "$SCRIPT_PATH" clear; then
        echo "OLED cleared successfully"
    else
        echo "Failed to clear OLED"
    fi
fi

sleep 0.01
