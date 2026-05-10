# BallBot

[![CI Testing](https://github.com/atticusrussell/ballbot/actions/workflows/ros-test.yaml/badge.svg)](https://github.com/atticusrussell/ballbot/actions/workflows/ros-test.yaml)
[![CI Linting](https://github.com/atticusrussell/ballbot/actions/workflows/ros-lint.yaml/badge.svg)](https://github.com/atticusrussell/ballbot/actions/workflows/ros-lint.yaml)

A 4WD differential-drive robot running ROS 2 Humble that detects pickleballs on a court, navigates to them, picks them up with an onboard 6-DOF arm, and returns them to a base station — all while staying outside the court boundary so it doesn't get stepped on.

The original v1 platform paired a Raspberry Pi 4 with a Teensy 4.1 (motor/encoder/IMU via [linorobot2](https://github.com/linorobot/linorobot2)), an RPLIDAR A1, and a USB webcam. The v2 redesign in progress swaps in a Jetson Orin Nano Super, a DOFBOT-SE 6-DOF arm, and an rpicam3.

**Status**: v2 in active development. See:
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — system design, node graph, TF tree, open architectural questions
- [`docs/ROADMAP.md`](docs/ROADMAP.md) — milestone breakdown
- [GitHub Project: Ballbot](https://github.com/users/atticusrussell/projects/6) — live progress

## Hardware

#### Front view
<p align='center'><img src=docs/images/20231024_200513.jpg></p>

#### Side view
<p align='center'><img src=docs/images/20231024_200719.jpg></p>

#### Rear top view
<p align='center'><img src=docs/images/20231024_200519.jpg></p>

#### Initial construction
<p align='center'><img src=docs/images/wip_catbot.jpg width="1000"></p>

### v1 part list

| | Part |
| --| --|
|1| [Raspberry Pi 4 (4 GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) — *being replaced by Jetson Orin Nano Super in v2*|
|2| [AmazonBasics 128 GB SD Card](https://www.amazon.com/dp/B08TJRVWV1)|
|3| [Yahboom Aluminum 4WD ROS Robot Car Chassis](https://category.yahboom.net/collections/a-chassis-bracket/products/ros-chassis)|
|4| [L298N Motor Drivers](https://www.amazon.com/dp/B07BK1QL5T)|
|5| [DFRobot DC-DC Power Module 25W DFR0205](https://www.digikey.com/en/products/detail/dfrobot/DFR0205/6588491)|
|6| [Screw-down Terminal Block Strips](https://www.amazon.com/dp/B08V4W637Q)|
|7| [RPLIDAR A1](https://www.slamtec.com/en/Lidar/A1)|
|8| [GeeekPi Fan Hat with OLED](https://www.amazon.com/dp/B09MVL8BWQ)|
|9| Teensy 4.1 (running [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware))|
|10| Various standoffs, wires, and crimped connectors|

### v2 additions (in progress)

- Jetson Orin Nano Super (compute upgrade)
- DOFBOT-SE 6-DOF arm + gripper (manipulation)
- Raspberry Pi Camera Module 3 (CSI, replacing the USB webcam)

See `ARCHITECTURE.md` for the full v2 hardware story including the open question on whether to bypass the DOFBOT STM32 expansion board and drive it directly over I2C from the Orin.

## Development

The project builds and runs in the VSCode devcontainers under `.devcontainer/`. Pick the variant matching your host:

- `linux-gpu/` — Linux host with NVIDIA GPU
- `wsl-gpu/` — WSL 2 with GPU passthrough

For GPU passthrough you'll first need the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Acknowledgments

- [Allison Thackston](https://github.com/athackst/vscode_ros2_workspace) — workspace template
- [Articulated Robotics](https://articulatedrobotics.xyz/) — tutorials
- [Lidarbot](https://github.com/TheNoobInventor/lidarbot)
- [linorobot2](https://github.com/linorobot/linorobot2) and [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) — base platform
- [ros2_rover](https://github.com/mgonzs13/ros2_rover/)
