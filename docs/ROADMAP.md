# BallBot v2 Roadmap

Milestone + issue breakdown for the v2 redesign. This is the working planning doc — refine here, don't re-dump in chat.

**Companion docs**: [`ARCHITECTURE.md`](./ARCHITECTURE.md) is the source of truth for system structure. This doc is the source of truth for *what gets built when*.

**Status legend**: 🟦 not started · 🟨 in progress · 🟩 done · ⬜ deferred / open

---

## Conventions

- Milestones are **flat integers**, zero-padded (`M01`–`M19`), so branches sort correctly in a terminal. No decimals, no `A`/`B` suffixes — every distinct chunk of work is its own integer.
- Each milestone gets a native GitHub Milestone; each bullet under "Issues" gets a native GitHub Issue assigned to it.
- **Build / hardware milestones** get a feature branch (`phase/M<id>-<slug>`) and are PR'd to `humble` at closure.
- **Theory milestones** (M04, M09, M12, M15) produce no code — no branch, no PR. They are a reading/watching checklist; close the GitHub Milestone when every resource issue is checked off and the **exit criterion** is met.
- Theory milestones gate the build milestone that consumes them — you don't use a technique you haven't studied. See the [Learning resources](#learning-resources) appendix for canonical links.
- Reading-checklist issues use the chapter PDFs in `docs/third_party/` and the video series in the appendix.
- Behavior tree (BT) extension issues live at the end of their parent build milestone.

---

## Milestone overview

| # | Milestone | Type |
|---|---|---|
| M01 | URDF foundation migration | build |
| M02 | v2 CAD design | build |
| M03 | HW Revival — diagnostic baseline | hardware |
| M04 | Theory: Localization & Filtering | theory |
| M05 | Sim Nav | build |
| M06 | v2 Hardware Build | hardware |
| M07 | Jetson Platform Bringup | hardware |
| M08 | Real Nav | build |
| M09 | Theory: Perception | theory |
| M10 | Sim Detector | build |
| M11 | Real Detector | build |
| M12 | Theory: Visual Odometry | theory |
| M13 | Sim VO + Court Keep-Out | build |
| M14 | Real VO at Court | build |
| M15 | Theory: Manipulation | theory |
| M16 | Sim Manipulation | build |
| M17 | Arm Commissioning | hardware |
| M18 | Real Manipulation | build |
| M19 | Final Integration + Court Demo | build |

---

## M01 — URDF foundation migration 🟨
**Goal**: Resurrect `archive/catbot_description`'s onshape-to-robot pipeline into `ballbot_description` with simplified collisions. Mass-accurate v1 robot in URDF (no v2 design changes yet).
**Branch**: `phase/M01-urdf-foundation`
**Issues**:
- Add `onshape-to-robot` pipeline to `ballbot_description` (config.json, `moveMeshes.sh`, `urdf_newlines.py`)
- Migrate CAD-derived chassis + wheel meshes from `archive/catbot_description/meshes/`
- Replace parametric base + wheel xacros with CAD-derived URDF macros
- Configure sensors (camera, lidar, IMU) via the onshape-to-robot pipeline's `additional.xml` block
- Simplify collision shapes: switch from full mesh collision to convex hull or primitive boxes (use the alternate forms already CAD'd)
- Verify URDF spawns in Gazebo with no self-collision
- Verify TF tree clean
- Confirm mass + CoG match v1 robot
- Delete `archive/catbot_description/` and `archive/catbot_simulation/` after merge (preserve `obstacles.world` first if absorbing)
---

## M02 — v2 CAD design 🟨
**Goal**: Update Onshape doc with v2 changes (arm mount, Jetson, depth cam placeholder), bump document version, rerun import. Should be "run the script" not "rebuild the system."
**Branch**: `phase/M02-v2-design`
**Issues**:
- Wire 4-pin I2C ribbon (Jetson 40-pin header I2C-7 → expansion board I2C header → STM8 coprocessor at addr `0x15`); replaces SE's USB-serial-via-CH340 path. See ARCHITECTURE.md decision log.
- Compose Yahboom DOFBOT-Pro arm URDF into BallBot URDF (skip re-CADing the arm)
- **Design top plate in Onshape**: arm mount + Jetson Orin Nano Super footprint + clearance for existing components. Must clear the arm's full sweep over the lidar and chassis camera (the arm gets bolted on permanently in M06)
- Add Jetson + heatsink as mass block in CAD
- Add depth camera as suppressed part (zero density, mounting holes only)
- Bump Onshape document version in `config.json`
- Rerun onshape-to-robot import
- Verify v2 URDF spawns in Gazebo, no self-collision
- Sanity-check inertia + CoG against estimated v2 numbers
---

## M03 — HW Revival — diagnostic baseline 🟦
**Goal**: Existing robot powers up in its current config with all ROS nodes green — a **known-good baseline established before any v2 teardown**. Diagnostic only: no hardware added or moved. Surfacing a dead battery or loose wire here gives procurement lead time before M06.
**Branch**: `phase/M03-hw-revival`
**Issues**:
- Inspect robot from storage for damage
- Test battery health + validate voltage rails; **order a replacement now if degraded** (lead time before M06)
- Verify Teensy firmware; re-flash with current `linorobot2_hardware` if needed (Teensy carries over to v2 unchanged)
- Boot RPi 4 in current config, restore wifi + ssh
- Diagnose + fix the cyclonedds env var issue
- Bringup smoke test: motors, lidar, IMU, camera all publish — **record this as the baseline node graph** (M07 reproduces it on the Jetson)
---

## M04 — Theory: Localization & Filtering 🟦
**Goal**: Understand state estimation, the EKF, and the particle filter before tuning `robot_localization` and AMCL. Theory milestone — no code.
**Exit criterion**: Can explain why AMCL uses a particle filter while `robot_localization` uses an EKF, what the "adaptive" in AMCL does (particle count shrinks once localized), and what each covariance term in the EKF config means.
**Issues** (see [Learning resources](#learning-resources) for links):
- 3Blue1Brown — *Essence of Linear Algebra* (finish the series) — the substrate under the EKF, PnP, and the servoing Jacobian
- MATLAB Tech Talks — *Understanding Sensor Fusion and Tracking*, Parts 1–3 only (Parts 4–6 are IMM / multi-object tracking — not on this roadmap)
- MATLAB Tech Talks — *Understanding Kalman Filters* (full series) — the EKF deep-dive; prereq for `robot_localization` (M05/M08) and VO fusion (M13)
- Cyrill Stachniss — *Mobile Robotics (Online Training)* — the Bayes filter → motion/observation models → KF/EKF → particle filter / Monte Carlo Localization block
- *Probabilistic Robotics* (Thrun/Burgard/Fox) — Ch 3 (Gaussian filters: KF/EKF), Ch 4 (nonparametric filters: particle filter), **Ch 8 (Monte Carlo Localization = AMCL)**

The theory ladder: Bayes filter → Gaussian filters (KF/EKF) → nonparametric filters (histogram + particle) → Monte Carlo Localization → AMCL.

---

## M05 — Sim Nav 🟦
**Goal**: 48/50 random nav goals succeed in multi-room sim apartment.
**Branch**: `phase/M05-sim-nav`
**Issues**:
- Build multi-room sim world (4-5 rooms, doorways, clutter)
- Update `ballbot_gazebo` launch to spawn v2 URDF in new world
- Configure nav2 params yaml for v2 footprint + dynamics
- Generate sim map via `slam_toolbox`
- Tune local + global costmap inflation
- Choose controller (DWB vs MPPI) — leaning MPPI given Orin compute
- Tune controller for v2 dynamics
- Implement 50-random-goal benchmark script
- Iterate tuning until ≥48/50
---

## M06 — v2 Hardware Build 🟦
**Goal**: The full v2 robot is physically assembled — top plate, Jetson, power, camera, **arm bolted on** — and powers on cleanly. The arm is mounted now (even though it's commissioned later in M17) so the **center of gravity locks here, once**, and never moves again. Build the body once; commission subsystems in sequence.
**Branch**: `phase/M06-v2-build`
**Issues**:
- **Fabricate v2 top plate** from the M02 Onshape design
- Mount Jetson + heatsink on top plate
- Install DC-DC converter; wire the Jetson power rail
- Mount rpicam3 (CSI ribbon routed)
- **Physically mount the DOFBOT arm on the top plate** — bolted, permanent; CoG locks here. Commissioning (servo cal, I2C) is deferred to M17
- Wire the 4-pin I2C ribbon (Jetson I2C-7 → expansion board → STM8 `0x15`)
- Retain / re-mount existing sensors (lidar, IMU)
- Power-on test: voltage rails good, nothing smokes
---

## M07 — Jetson Platform Bringup 🟦
**Goal**: The baseline node graph comes up green on the Jetson — **the same smoke test M03 passed on the RPi 4**. This parity check proves the platform migration and isolates it from nav tuning (M08).
**Branch**: `phase/M07-jetson-bringup`
**Issues**:
- **Decide Jetson base image**: clean JetPack 6.2 + ROS 2 Humble (recommended) vs Yahboom vendor image — see ARCHITECTURE.md §8 Q13
- Flash JetPack 6.2; install ROS 2 Humble
- Migrate ROS 2 workspace to Jetson, build all `ballbot_*` packages on aarch64
- micro-ROS agent for the Teensy on the Jetson (Teensy firmware unchanged)
- rplidar driver over USB
- rpicam3: verify libcamera + ROS image publisher (CSI)
- Verify IMU data publishes via the Teensy
- Verify the FastDDS config (`ballbot_base/config/fastrtps.xml`) on the Jetson
- Restore wifi + ssh
- Smoke test: **baseline node graph green on Jetson — parity with the M03 baseline**
---

## M08 — Real Nav 🟦
**Goal**: ≥9/10 RViz goals succeed in your real apartment. Tuned once, on the final v2 platform — no re-tune, because there is no compute swap or CoG change after this point.
**Branch**: `phase/M08-real-nav`
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

## M09 — Theory: Perception 🟦
**Goal**: Understand machine learning, neural networks, and the camera model before training a detector. Theory milestone — no code.
**Exit criterion**: Can explain a convolutional layer, what YOLO actually regresses, what mAP measures, and why a held-out test set + domain randomization matter (the "why" behind half of M10's task list).
**Issues** (see [Learning resources](#learning-resources) for links):
- Welch Labs — *Learning to See* (~2h) — the ML mindset: overfitting, generalization, why evaluation is hard. **Watch this first.**
- 3Blue1Brown — *Neural Networks / Deep Learning*, chapters 1–4
- Welch Labs — *Neural Networks Demystified* — the same, with the implementation + calculus
- Welch Labs — long-form deep-learning videos (AlexNet, "Why Deep Learning Works Unreasonably Well") — why a *convolutional* detector works
- DOFBOT course 07 (OpenCV) — 18-chapter checklist (`docs/third_party/dofbot-tutorials/`)
- DOFBOT course 08 (AI vision basic) — 6-chapter checklist
- First Principles of Computer Vision (Shree Nayar) — image formation + the camera model

---

## M10 — Sim Detector 🟦
**Goal**: ≥90% mAP on synthetic test set.
**Branch**: `phase/M10-sim-detector`
**Issues**:
- Spawn pickleballs in Gazebo with randomized poses
- Implement domain randomization (lighting, court color, distractors)
- Build auto-label image capture pipeline (1-2k images)
- Set up YOLO training env on the RTX 3080
- Train YOLOv8 (size: m or l, given Orin compute) on synthetic dataset
- Eval on held-out synthetic test set, iterate to ≥90% mAP
- Sanity-test detector node live in Gazebo
---

## M11 — Real Detector 🟦
**Goal**: ≥30fps on Orin, ≥90% precision in apartment 0.5-3m range.
**Branch**: `phase/M11-real-detector`
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

## M12 — Theory: Visual Odometry 🟦
**Goal**: Understand line/feature detection, optical flow, and pose-from-geometry before building VO. Theory milestone — no code.
**Exit criterion**: Can explain PnP — how known 3D points plus their 2D image projections solve for camera pose — and how line correspondences against a known court layout become a registration problem.
**Issues** (see [Learning resources](#learning-resources) for links):
- DOFBOT course 12 (ROS+OpenCV) — Hough lines, edge, contour, feature tracking, optical flow chapters (`docs/third_party/`)
- Cyrill Stachniss — *Mobile Sensing and Robotics 2* — visual features, RANSAC, camera geometry, DLT, P3P, epipolar geometry
- First Principles of Computer Vision — feature detection, optical flow, camera calibration, pose estimation lectures
- *Computer Vision: Algorithms and Applications* (Szeliski) — feature detection/matching + structure-from-motion chapters (free, legal — see appendix)
- *Multiple View Geometry in Computer Vision* (Hartley & Zisserman) — the PnP chapter

---

## M13 — Sim VO + Court Keep-Out 🟦
**Goal**: <10cm pose error over 20m sim court traversal; keep-out costmap working.
**Branch**: `phase/M13-sim-vo`
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

## M14 — Real VO at Court 🟦
**Goal**: <30cm drift on real court; field-trip data captured.
**Branch**: `phase/M14-real-vo`
**Issues**:
- Field trip 1: record bag of camera/lidar/odom on real court
- Run VO pipeline offline against bag, plot trajectory
- Validate <30cm drift over court length
- Tune line detector for real lighting/shadows
- Field trip 2: live VO at court
- **BT v2**: BT v1 + enforce court keep-out polygon (via `FilterBallsOutsideCourt` BT node + nav2 keepout filter)
- Validate BT v2 with 3 cycles outside boundary
---

## M15 — Theory: Manipulation 🟦
**Goal**: Kinematics, control, and MoveIt2 fundamentals before manipulation. Theory milestone — no code.

Prefer DOFBOT-Pro Orin-Super tutorials (ROS 2 native, MoveIt2) over DOFBOT-SE (ROS 1) where chapters overlap.

**Exit criterion**: Can explain forward vs inverse kinematics, the manipulator Jacobian, and IBVS vs PBVS (closes ARCHITECTURE.md §8 Q2).
**Issues** (see [Learning resources](#learning-resources) for links):
- *Modern Robotics* (Lynch & Park, Northwestern) — book + YouTube lecture series (free, legal) — forward/inverse kinematics, the Jacobian, trajectories
- Read DOFBOT-Pro `23.For_JetsonORIN_SUPER_JetPack6.2/12. MoveIt Case Study/` — full 10-chapter checklist (MoveIt2 config, IK design, trajectory planning, collision detection)
- Read DOFBOT-Pro `23.For_JetsonORIN_SUPER_JetPack6.2/21. ROS2 basic course/21.ROS2 URDF model.pdf`
- Read DOFBOT-SE course 06 (Basic control) — 12-chapter checklist (servo control fundamentals, hardware-level)
- Read DOFBOT-SE course 09 (AI vision tracking, ch 1: PID basics) — single-chapter focus
- Read DOFBOT-Pro course 11 (3D Robotic Arm Control and Inverse Kinematics) — 2-chapter checklist

---

## M16 — Sim Manipulation (eye-in-hand mono) 🟦
**Goal**: Sim arm grasps ball ≥80% in sim.
**Branch**: `phase/M16-sim-manip`
**Issues**:
- Add eye-in-hand camera to URDF on arm end-effector
- Verify camera publishes in Gazebo
- Implement visual servoing controller (IBVS or PBVS — pick + justify)
- Configure hand-eye calibration in sim
- Implement grasp policy (approach pose → close gripper)
- Spawn 50 random-pose balls, log success rate
- Iterate to ≥80%
---

## M17 — Arm Commissioning 🟦
**Goal**: The arm — already physically mounted in M06 — is electrically commissioned and calibrated. **Commissioning only: no physical mounting, so zero CoG impact and nav is untouched.**
**Branch**: `phase/M17-arm-commissioning`
**Issues**:
- Run servo calibration sequence
- Verify arm joints respond to ROS commands over I2C (per M02 decision)
- Verify reachable workspace matches URDF
- Mount eye-in-hand camera on the arm end-effector
- Hand-eye calibration on the real arm
- Smoke test: arm moves to a commanded Cartesian pose
---

## M18 — Real Manipulation 🟦
**Goal**: Real arm grasps real pickleball ≥80% on bench.
**Branch**: `phase/M18-real-manip`
**Issues**:
- Re-tune visual servoing for real arm dynamics
- Bench: 50 grasp attempts on real ball at known poses
- Iterate to ≥80%
- **BT v3**: BT v2 + grasp + drop at base
- Validate BT v3 with 3 successful pickup-and-drop cycles
---

## M19 — Final Integration + Court Demo 🟦
**Goal**: ≥3 balls picked at real court, demo video captured.
**Branch**: `phase/M19-court-demo`
**Issues**:
- Full pipeline sim test: 3-ball court demo end-to-end
- Field trip 3: full pipeline at real court
- Iterate on real-world failure modes
- Capture demo video (multi-angle, edited)
- Write project page / portfolio post
---

## Learning resources

Canonical links for the theory milestones (M04, M09, M12, M15).

### Video series

- **3Blue1Brown — Essence of Linear Algebra** — the substrate under every filter and geometry topic. <https://www.youtube.com/watch?v=fNk_zzaMoSs>
- **3Blue1Brown — Neural Networks / Deep Learning** (start at chapter 1). <https://www.youtube.com/watch?v=aircAruvnKk>
- **Welch Labs** — *Learning to See* (ML mindset), *Neural Networks Demystified*, and the long-form deep-learning videos. <https://www.youtube.com/@WelchLabsVideo>
- **MATLAB Tech Talks — Understanding Sensor Fusion and Tracking** — Parts 1–3 only.
- **MATLAB Tech Talks — Understanding Kalman Filters** — full series. <https://www.youtube.com/playlist?list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr>
- **Cyrill Stachniss — Mobile Robotics (Online Training)** (Uni Bonn) — Bayes filter, motion/observation models, KF/EKF, particle filter / MCL (M04). <https://www.youtube.com/playlist?list=PLgnQpQtFTOGSeTU35ojkOdsscnenP2Cqx>
- **Cyrill Stachniss — Mobile Sensing and Robotics 2** (Uni Bonn) — visual features, RANSAC, camera geometry, DLT, P3P, epipolar geometry (M12). <https://www.youtube.com/playlist?list=PLgnQpQtFTOGQh_J16IMwDlji18SWQ2PZ6>
- **First Principles of Computer Vision** (Shree Nayar, Columbia) — image formation, features, pose estimation. <https://www.youtube.com/channel/UCf0WB91t8Ky6AuYcQV0CcLw> · <https://fpcv.cs.columbia.edu/>
- **Modern Robotics** (Lynch & Park, Northwestern) — kinematics, Jacobians, trajectories — book + YouTube lectures. <http://hades.mech.northwestern.edu/index.php/Modern_Robotics>

### Books

| Book | Sourcing |
|---|---|
| *Probabilistic Robotics* — Thrun, Burgard, Fox | Paid (MIT Press, ISBN 978-0262201629). PDF on Library Genesis / Anna's Archive, or a university library. |
| *Computer Vision: Algorithms and Applications* — Szeliski | **Free & legal** — full PDF at <https://szeliski.org/Book/> |
| *Modern Robotics* — Lynch & Park | **Free & legal** — PDF + lectures at the Northwestern link above |
| *Deep Learning* — Goodfellow, Bengio, Courville | **Free & legal** — <https://www.deeplearningbook.org/> |
| *Multiple View Geometry in Computer Vision* — Hartley & Zisserman | Paid — library or PDF |

---

## Tutorial reading sources

- DOFBOT-SE tutorials (`docs/third_party/dofbot-se/`, also a vendored copy at `docs/third_party/dofbot-tutorials/`) are **ROS 1**. Useful for fundamentals (PID, OpenCV, Linux), less useful for ROS 2 specifics.
- DOFBOT-Pro tutorials (`docs/third_party/dofbot-pro/`) include `23.For_JetsonORIN_SUPER_JetPack6.2/` with **ROS 2 native** content: MoveIt2 configuration, MoveIt2 inverse kinematics, MoveIt2 trajectory planning, ROS 2 URDF model.
- **M15 (Theory: Manipulation) prefers the Pro/Orin-Super tutorials** for MoveIt2 + ROS 2 chapters, falling back to SE only for fundamentals.
- Yahboom-provided source code (URDFs, ROS nodes) is at `third_party/dofbot-pro/` (Drive download) and `third_party/dofbot-se/` — separate from the docs.

## Side-quest issues

Off-roadmap fill-in work lives in GitHub issues with the `side-quest` label, no milestone, not on the project board. Pull from there when looking for a small task between phase commitments. See [open side-quests](https://github.com/atticusrussell/ballbot/issues?q=is%3Aopen+is%3Aissue+label%3Aside-quest).
