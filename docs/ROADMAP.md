# BallBot v2 Roadmap

Milestone + issue breakdown for the v2 redesign. This is the working planning doc — refine here, don't re-dump in chat.

**Companion docs**: [`ARCHITECTURE.md`](./ARCHITECTURE.md) is the source of truth for system structure. This doc is the source of truth for *what gets built when*.

**Status legend**: 🟦 not started · 🟨 in progress · 🟩 done · ⬜ deferred / open

---

## Conventions

- Each milestone gets a GitHub Milestone (native).
- Each bullet under "Issues" gets a GitHub Issue (native), tagged with the milestone.
- Each milestone gets its own feature branch (`phase/M<id>-<slug>`), PR'd to `humble` when the milestone closes.
- **Each milestone closure gets a git tag**: `m{id}-complete` (e.g., `m0.1-complete`, `m1a-complete`, `m4b-complete`). Tag every milestone, including prep / reading / hardware milestones, for trivial rollback to any phase boundary.
- Reading-group issues use a checklist of chapter PDFs from `docs/third_party/dofbot-tutorials/`.
- Behavior tree (BT) extension issues live at the end of their parent "B" milestone.
- HW integration milestones (M*.5) are atomic and small — green-light test only.

---

## M0.1 — URDF foundation migration 🟦
**Goal**: Resurrect `archive/catbot_description`'s onshape-to-robot pipeline into `brobot_description` with simplified collisions. Mass-accurate v1 robot in URDF (no v2 design changes yet).
**Branch**: `phase/M0.1-urdf-foundation`
**Issues**:
- Add `onshape-to-robot` pipeline to `brobot_description` (config.json, `moveMeshes.sh`, `urdf_newlines.py`)
- Migrate CAD-derived chassis + wheel meshes from `archive/catbot_description/meshes/`
- Replace parametric base + wheel xacros with CAD-derived URDF macros
- Keep brobot's modular sensor xacros (camera, laser, imu) — wire them into the new base
- Simplify collision shapes: switch from full mesh collision to convex hull or primitive boxes (use the alternate forms already CAD'd)
- Verify URDF spawns in Gazebo with no self-collision
- Verify TF tree clean
- Confirm mass + CoG match v1 robot
- Delete `archive/catbot_description/` and `archive/catbot_simulation/` after merge (preserve `obstacles.world` first if absorbing)
- Open PR + tag `m0.1-complete`

---

## M0.2 — v2 CAD design changes 🟦
**Goal**: Update Onshape doc with v2 changes (arm mount, Jetson, depth cam placeholder), bump document version, rerun import. Should be "run the script" not "rebuild the system."
**Branch**: `phase/M0.2-v2-design`
**Issues**:
- **Investigate STM32 bypass: direct I2C from Orin to expansion board** (study DOFBOT-Pro firmware + URDF; decide before CAD changes since it affects cable routing + plate cutouts)
- Compose Yahboom DOFBOT-Pro arm URDF into BallBot URDF (skip re-CADing the arm)
- Design top plate in Onshape: arm mount + Jetson Orin Nano Super footprint + clearance for existing components
- Add Jetson + heatsink as mass block in CAD
- Add depth camera as suppressed part (zero density, mounting holes only)
- Bump Onshape document version in `config.json`
- Rerun onshape-to-robot import
- Verify v2 URDF spawns in Gazebo, no self-collision
- Sanity-check inertia + CoG against estimated v2 numbers
- Open PR + tag `m0.2-complete`

---

## M1A — Sim Nav 🟦
**Goal**: 48/50 random nav goals succeed in multi-room sim apartment.
**Branch**: `phase/M1A-sim-nav`
**Issues**:
- Build multi-room sim world (4-5 rooms, doorways, clutter)
- Update `brobot_gazebo` launch to spawn v2 URDF in new world
- Configure nav2 params yaml for v2 footprint + dynamics
- Generate sim map via `slam_toolbox`
- Read AMCL / particle filter background (Probabilistic Robotics ch 8 or equivalent)
- Tune local + global costmap inflation
- Choose controller (DWB vs MPPI) — leaning MPPI given Orin compute
- Tune controller for v2 dynamics
- Implement 50-random-goal benchmark script
- Iterate tuning until ≥48/50

---

## M1.5 — HW Revival 🟦
**Goal**: Existing real robot powers up, all ROS nodes green.
**Branch**: `phase/M1.5-hw-revival`
**Issues**:
- Inspect robot from storage for damage
- Test/replace battery, validate voltage rail
- Re-flash Teensy with current `linorobot2_hardware`
- Boot RPi 4 (interim host), restore wifi + ssh
- Diagnose and fix the cyclonedds env var issue
- Bringup smoke test (motors, lidar, IMU, camera all publish)

---

## M1B — Real Nav 🟦
**Goal**: ≥9/10 RViz goals succeed in your real apartment.
**Branch**: `phase/M1B-real-nav`
**Issues**:
- Run `slam_toolbox` in apartment, save initial map
- Extend map to multi-room coverage
- Configure AMCL with saved map
- Re-tune costmaps for real RPLIDAR A1 noise
- Run 10-goal apartment benchmark from RViz
- Iterate until ≥9/10
- **BT v0**: nav-to-hardcoded-pose + return-to-base + handle nav failure
- Validate BT v0 with 5 successful cycles

---

## Reading-A — Pre-Detector 🟦
**Goal**: Foundational CV reading before training a detector.
**Issues**:
- Read DOFBOT course 07 (OpenCV course) — 18-chapter checklist
- Read DOFBOT course 08 (AI vision basic) — 6-chapter checklist

---

## M2A — Sim Detector 🟦
**Goal**: ≥90% mAP on synthetic test set.
**Branch**: `phase/M2A-sim-detector`
**Issues**:
- Spawn pickleballs in Gazebo with randomized poses
- Implement domain randomization (lighting, court color, distractors)
- Build auto-label image capture pipeline (1-2k images)
- Set up YOLO training env on the RTX 3080
- Train YOLOv8 (size: m or l, given Orin compute) on synthetic dataset
- Eval on held-out synthetic test set, iterate to ≥90% mAP
- Sanity-test detector node live in Gazebo

---

## M2.5 — HW Compute + Cam 🟦
**Goal**: Jetson Orin Nano Super + rpicam3 swap, dummy inference passes.
**Branch**: `phase/M2.5-hw-compute-cam`
**Issues**:
- Acquire Jetson Orin Nano Super carrier + PSU (if not ready)
- Acquire rpicam3 (if not ready)
- Mount Jetson on new top plate
- Wire Jetson power (rail or independent BEC)
- Migrate ROS workspace to Jetson, build all packages
- Connect rpicam3 via CSI, verify libcamera + ROS image publisher
- Smoke test: detector node runs on Jetson with live cam

---

## M2B — Real Detector 🟦
**Goal**: ≥30fps on Orin, ≥90% precision in apartment 0.5-3m range.
**Branch**: `phase/M2B-real-detector`
**Issues**:
- Capture 200-500 real pickleball photos (varied lighting)
- Fine-tune sim-trained model on real data
- Convert model to TensorRT
- Deploy detector node on Orin, measure fps
- Validate ≥90% precision in apartment
- Implement detection_projector node (2D pixel → 3D ground-plane pose)
- **BT v1**: detect → nav-to-ball → return
- Validate BT v1 with 5 cycles in apartment

---

## Reading-B — Pre-VO 🟦
**Goal**: Line/feature/optical-flow reading before VO.
**Issues**:
- Read DOFBOT course 12 (ROS+OpenCV) — selected chapters checklist (Hough lines, edge, contour, feature tracking, optical flow)

---

## M3A — Sim VO + Court Keep-Out 🟦
**Goal**: <10cm pose error over 20m sim court traversal; keep-out costmap working.
**Branch**: `phase/M3A-sim-vo`
**Issues**:
- Build sim pickleball court world (lines, fences)
- Implement line detection node (Canny + Hough)
- Implement court geometry matcher (detected lines → known layout)
- Solve PnP for camera pose against court
- Fuse VO with wheel odometry via `robot_localization` EKF
- Validate <10cm pose error over 20m sim traversal
- Publish court polygon as nav2 static-layer costmap (keep-out filter)
- Verify nav2 respects keep-out in sim

---

## M3B — Real VO at Court 🟦
**Goal**: <30cm drift on real court; field-trip data captured.
**Branch**: `phase/M3B-real-vo`
**Issues**:
- Field trip 1: record bag of camera/lidar/odom on real court
- Run VO pipeline offline against bag, plot trajectory
- Validate <30cm drift over court length
- Tune line detector for real lighting/shadows
- Field trip 2: live VO at court
- **BT v2**: BT v1 + enforce court keep-out polygon (via `FilterBallsOutsideCourt` BT node + nav2 keepout filter)
- Validate BT v2 with 3 cycles outside boundary

---

## Reading-C — Pre-Manipulation 🟦
**Goal**: Kinematics, control, MoveIt2 fundamentals before manipulation.

Prefer DOFBOT-Pro Orin-Super tutorials (ROS 2 native, MoveIt2) over DOFBOT-SE (ROS 1) where chapters overlap.

**Issues**:
- Read DOFBOT-Pro `23.For_JetsonORIN_SUPER_JetPack6.2/12. MoveIt Case Study/` — full 10-chapter checklist (MoveIt2 config, IK design, trajectory planning, collision detection)
- Read DOFBOT-Pro `23.For_JetsonORIN_SUPER_JetPack6.2/21. ROS2 basic course/21.ROS2 URDF model.pdf`
- Read DOFBOT-SE course 06 (Basic control) — 12-chapter checklist (servo control fundamentals, hardware-level)
- Read DOFBOT-SE course 09 (AI vision tracking, ch 1: PID basics) — single-chapter focus
- Read DOFBOT-Pro course 11 (3D Robotic Arm Control and Inverse Kinematics) — 2-chapter checklist

---

## M4A — Sim Manipulation (eye-in-hand mono) 🟦
**Goal**: Sim arm grasps ball ≥80% in sim.
**Branch**: `phase/M4A-sim-manip`
**Issues**:
- Add eye-in-hand camera to URDF on arm end-effector
- Verify camera publishes in Gazebo
- Implement visual servoing controller (IBVS or PBVS — pick + justify)
- Configure hand-eye calibration in sim
- Implement grasp policy (approach pose → close gripper)
- Spawn 50 random-pose balls, log success rate
- Iterate to ≥80%

---

## M4.5 — HW Arm 🟦
**Goal**: Arm physically mounted, calibrated, reachable.
**Branch**: `phase/M4.5-hw-arm`
**Issues**:
- Physically mount DOFBOT-SE (or bypassed config from M0.2 decision) to top plate
- Run servo calibration sequence
- Verify arm joints respond to ROS commands (over USB or I2C per M0.2 decision)
- Verify reachable workspace matches URDF
- Mount eye-in-hand camera on arm
- Hand-eye calibration on real arm
- Smoke test: arm moves to commanded Cartesian pose

---

## M4B — Real Manipulation 🟦
**Goal**: Real arm grasps real pickleball ≥80% on bench.
**Branch**: `phase/M4B-real-manip`
**Issues**:
- Re-tune visual servoing for real arm dynamics
- Bench: 50 grasp attempts on real ball at known poses
- Iterate to ≥80%
- **BT v3**: BT v2 + grasp + drop at base
- Validate BT v3 with 3 successful pickup-and-drop cycles

---

## M5 — Final Integration + Court Demo 🟦
**Goal**: ≥3 balls picked at real court, demo video captured.
**Branch**: `phase/M5-court-demo`
**Issues**:
- Full pipeline sim test: 3-ball court demo end-to-end
- Field trip 3: full pipeline at real court
- Iterate on real-world failure modes
- Capture demo video (multi-angle, edited)
- Write project page / portfolio post

---

## Tutorial reading sources

- DOFBOT-SE tutorials (`docs/third_party/dofbot-se/`, also a vendored copy at `docs/third_party/dofbot-tutorials/`) are **ROS 1**. Useful for fundamentals (PID, OpenCV, Linux), less useful for ROS 2 specifics.
- DOFBOT-Pro tutorials (`docs/third_party/dofbot-pro/`) include `23.For_JetsonORIN_SUPER_JetPack6.2/` with **ROS 2 native** content: MoveIt2 configuration, MoveIt2 inverse kinematics, MoveIt2 trajectory planning, ROS 2 URDF model.
- **Reading-C (Pre-Manipulation) prefers the Pro/Orin-Super tutorials** for MoveIt2 + ROS 2 chapters, falling back to SE only for fundamentals.
- Yahboom-provided source code (URDFs, ROS nodes) is at `third_party/dofbot-pro/` (Drive download) and `third_party/dofbot-se/` — separate from the docs.

## Side-quest issues

Off-roadmap fill-in work lives in GitHub issues with the `side-quest` label, no milestone, not on the project board. Pull from there when looking for a small task between phase commitments. See [open side-quests](https://github.com/atticusrussell/ballbot/issues?q=is%3Aopen+is%3Aissue+label%3Aside-quest).
