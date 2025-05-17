# 🛠 TODO

---

## 🧰 Hardware

- [x] Interface with motors  
- [x] Finish chassis wiring  
- [x] Integrate LIDAR  
- [x] Integrate camera  
- [x] Replace DC-DC converter  
- [x] Add battery voltage display / low-voltage cutoff  
- [x] Replace wiring from buck converter to RPi (dropping 200mV — causing undervoltage)  
  - [x] stress test

- [ ] Add actuator / gripper or scoop mechanism  
- [ ] Evaluate adding depth camera (for ball/person detection or terrain nav)  
- [ ] Evaluate compute requirements — upgrade to Jetson if needed  

- [ ] Figure out using RPi Camera Module 3 instead of webcam  
  - [ ] Investigate [libcamera integration](https://github.com/codermery/camera_ros)  
  - [ ] Try patching kernel for Sony sensor ([ref](https://github.com/raspberrypi/rpicam-apps/issues/551)) — no success before  

- [ ] integrate battery voltage with ROS? 

- [ ] investigate robot shell / durability / surviving tennis ball direct hit

---

## 📝 Documentation

- [x] Add script and documents for basic Ubuntu config post-install
- [x] Update TODO regarding tennis ball
- [x] Add documentation and scripts for installing/configuring peripherals  
  - [x] Setup OLED Pi HAT and fan  
    - [x] Can this show voltage too?  
  - [x] Setup LIDAR  
  - [x] Setup webcam  
  - [ ] Setup Teensy with `linorobot2_hardware`  

- [ ] Document ROS installation and dependencies  
- [ ] Document `linorobot2` dependencies — copy usage instructions  
  - [ ] Make it clear this repo is largely a fork of `linorobot2`  
- [ ] Add backup/restore instructions  
- [ ] Document ROS autostart on boot  
- [ ] Update README to explain the tennis-ball-tracking goal  
- [ ] Update docs to reflect real components  
  - Teensy 4.1  
  - Link to `linorobot2_hardware`  
    - [ ] Consider using submodule  
    - [ ] Add script to pull/build in top-level repo  

---

## 🎾 Tennis Ball Goal

- [x] Recognize tennis ball (CV pipeline)  
- [ ] Locate tennis ball (3D if possible — depth camera?)  
- [ ] Plan path to tennis ball  
- [ ] Navigate to tennis ball using Nav2  
- [ ] Add actuator/gripper or scoop mechanism  
- [ ] Pickup tennis ball  

- [ ] Recognize person  
- [ ] Navigate to person  
- [ ] Deliver tennis ball  

- [ ] Consider alternatives to person delivery (base station at court edge?)  
- [ ] Evaluate ball delivery logic — person, goalpost, or bin  

---

## 🤖 ROS & Simulation

- [x] Simulate robot in Gazebo  
- [x] Teleoperate robot with keyboard
- [x] Use Nav2 in simulation - linorobot2 version
- [ ] Modify Nav2 launch files  
- [ ] Use Nav2 with costmap and goal pose in simulation  
- [ ] Test Nav2 with waypoints in simulation  
- [ ] Investigate Nav2 AMCL  
- [ ] Fix Nav2 spinning behavior (disable rotate-to-goal)  
- [ ] Run Nav2 on the real robot  
- [ ] Do full loop of house using Nav2 waypoints  
- [ ] Integrate gamepad control  
- [ ] Implement tennis ball detection using OpenCV (or similar)  

---

## 🧪 Side Quests 
- [x] Fix `linorobot2_hardware` ROS versions and add CI  
- [x] Convert CAD to detailed URDF  

## Technical Maintenance
- [ ] Upgrade to latest Gazebo version  
- [ ] Pull in updates from upstream `linorobot2`  - nav improvements?
- [ ] Pull in updates from upstream `linorobot2_hardware`  
- [ ] Consider submoduling a fork of `linorobot2` for easier upstream sync  
