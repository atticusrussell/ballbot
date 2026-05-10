#!/bin/bash

sudo systemctl start yahboom_discover.service

cd /home/jetson/app_Arm
python3 /home/jetson/app_Arm/YahboomArm.pyc


