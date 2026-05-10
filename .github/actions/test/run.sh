#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

./setup.sh
./build.sh
./test.sh
