# BroBot

![CI Testing](https://github.com/atticusrussell/catbot/actions/workflows/.github/workflows/ros-test.yaml/badge.svg)
![CI Linting](https://github.com/atticusrussell/catbot/actions/workflows/.github/workflows/ros-lint.yaml/badge.svg)


A 4WD differential drive robot is controlled using ROS2 Humble running on a Raspberry Pi 4 (running Ubuntu server 22.04). The vehicle is equipped with a camera for visual feedback and an RPLIDAR A1 sensor used for Simultaneous Localization and Mapping (SLAM), autonomous navigation and obstacle avoidance. The Linorobot2 project is leveraged through my use of a Teensy 4.1 running a Micro-ROS node to interface with 4 motors/encoders and an IMU.

The intent of the project is to explore robotics and computer vision by developing a robot that can detect a tennis ball, navigate toward it, and eventually return it to a person or designated base station — like a ball-retrieving assistant.


See [the workspace template](/template.md) for workspace usage instructions.


***(Work in Progress)***

## Tasks 
➡️ See the full task breakdown in [docs/TODO.md](docs/TODO.md)

## Hardware
####  BroBot Front View
<p align='center'>
    <img src=docs/images/20231024_200513.jpg>
</p>

####  BroBot Side View
<p align='center'>
    <img src=docs/images/20231024_200719.jpg>
</p>

####  BroBot Rear Top View
<p align='center'>
    <img src=docs/images/20231024_200519.jpg>
</p>

#### BroBot Initial Construction
<p align='center'>
    <img src=docs/images/wip_catbot.jpg width="1000">
</p>

### Part list
The following components were used in this project:

| | Part |
| --| --|
|1| [Raspberry Pi 4 (4 GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)|
|2| [AmazonBasics 128 GB SD Card](https://www.amazon.com/dp/B08TJRVWV1?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|3| [Yahboom Aluminum Alloy ROS Robot Car Chassis (4wd chassis)](https://category.yahboom.net/collections/a-chassis-bracket/products/ros-chassis)|
|4| [L298N Motor Drivers](https://www.amazon.com/dp/B07BK1QL5T?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|5| [DFRobot DC-DC Power Module 25W DFR0205](https://www.digikey.com/en/products/detail/dfrobot/DFR0205/6588491)|
|6| [Screw-down Terminal Block Strips Dual Row 10A 380V](https://www.amazon.com/dp/B08V4W637Q?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|7| [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)|
|8| [GeeekPi Fan Hat with OLED for RPi 4/3/2/B/+](https://www.amazon.com/dp/B09MVL8BWQ?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|9| [GeeekPi M2.5 Standoffs](https://www.amazon.com/dp/B07PHBTTGV?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|10| [Dupont Wires](https://www.amazon.com/dp/B01EV70C78?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|11| Arduino Uno|
|12| Spare wires|

Some other tools or parts used in the project are as follows:

| | Tool/Part |
| --| --|
|1| [Soldering iron](https://www.amazon.com/gp/product/B00ANZRT4M/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)|
|2| [SOMELINE Ferrule Crimping Tool Kit](https://www.amazon.com/dp/B09FSWKRH5?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|3| Screwdriver set|
|4| [Hot Glue Gun](https://www.amazon.com/dp/B00FI6QWBM?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|5| [Hot Glue](https://www.amazon.com/dp/B06X1CZWC5?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|6| [iCrimp IWS-3220M Micro Connector Pin Crimping Tool](https://www.amazon.com/dp/B078WPT5M1?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|7| [Connector Crimp Pin Cable Kit JST SYP Futaba](https://www.amazon.com/dp/B09MYWTHDZ?psc=1&ref=ppx_yo2ov_dt_b_product_details)|
|8| Zip ties |

## Instructions
### Devcontainers
The project can be built and developed in the VSCode devcontainers in `.devcontainer` .

For the devcontainers with GPU passthrough enabled, you first must [install the nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), either on Linux or on WSL2

## Acknowledgments
- [Allison Thackston](https://github.com/athackst/vscode_ros2_workspace)
- [Articulated Robotics](https://articulatedrobotics.xyz/)
- [Lidarbot](https://github.com/TheNoobInventor/lidarbot)
- [linorobot2](https://github.com/linorobot/linorobot2)
- [linrobot2_hardware](https://github.com/linorobot/linorobot2_hardware)
- [ros2_rover](https://github.com/mgonzs13/ros2_rover/)
