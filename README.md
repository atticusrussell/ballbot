# Catbot

![ROS2 CI](https://github.com/atticusrussell/catbot/actions/workflows/.github/workflows/ros.yaml/badge.svg)

A 4WD differential drive robot is controlled using ROS2 Humble running on a Raspberry Pi 4 (running Ubuntu server 22.04). The vehicle will be equipped with a camera for visual feedback and an RPLIDAR A1 sensor used for Simultaneous Localization and Mapping (SLAM), autonomous navigation and obstacle avoidance. The intent of the project is to learn about robotics, and to eventually recognize my cat's face and chase him to alleviate some boredom.

See [the workspace template](/template.md) for usage instructions.


***(Work in Progress)***

## Tasks 
(non-exhaustive)
- [x] Convert CAD to URDF
- [ ] Simulate in Gazebo
- [ ] Finish Chassis Construction
- [ ] Interface with motors [through arduino](https://github.com/joshnewans/ros_arduino_bridge) and [integrate with ros2_control](https://github.com/joshnewans/diffdrive_arduino/tree/humble)
- [ ] Integrate LIDAR
- [ ] Purchase + integrate camera
- [ ] Chase cat

## Hardware
#### Robot Under Construction
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

## Acknowledgments
- [Allison Thackston](https://github.com/athackst/vscode_ros2_workspace)
- [Articulated Robotics](https://articulatedrobotics.xyz/)
- [Lidarbot](https://github.com/TheNoobInventor/lidarbot)

#### Cricket
<p align='center'>
    <img src=docs/images/cricket.jpg width="1000">
</p>
