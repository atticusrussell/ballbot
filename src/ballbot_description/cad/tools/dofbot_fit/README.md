# dofbot_fit

Scripts for placing the YB-EVV01 PCB + Orin in `dofbot_pcb_housing` like the DOFBOT Pro CAD, and designing the
printable housing. Data (STEP exports, point clouds, transforms) lives in gitignored `cad/context/dofbot_fit/`.

Run with `~/.venvs/ballbot-cad/bin/python <script>.py` (needs matplotlib + scipy in that venv).

| Script | What |
|---|---|
| `crop.py` | Crop the fused DOFBOT STEP to faces near the enclosure → `region.brep` |
| `render.py`, `render_bottom.py`, `render_parts.py` | Matplotlib previews |
| `planes.py`, `cyl.py`, `mount.py` | List big planar faces / holes in the fused region |
| `step1b.py` | Exact fused→housing transform from the shell (vertex match over 24 rotations) |
| `step1_shell.py` | Shell registration (first pass) |
| `orin_pose.py` | Orin pose from the 86x58 hole pattern + 20 mm gap; 0/180 picked by fused overlap → `D_orin.npy` |
| `orin_top.py` | Tallest Orin solids in frame L (fan centre, top clearance) |
| `check.py`, `check_top.py`, `base_pcb.py` | Hole alignment + interference checks (boards, cover, A1, bridge, pins) |
| `place.py` | Onshape: insert Orin, push `out/orin_housing.step`, insert housing/A1/bridge, housing into chassis (~30 calls, resumable via `placed.json`) |
| `repair.py`, `repair_pcb.py`, `reset_positions.py` | One-off repairs of the first placement (relative re-inserts) |
| `fix_kit.py` | (superseded by `lidar_swap.py`) Full A1 dev-kit stack (devboard, connector, 15/8 mm stands) at kit positions on the cover pads |
| `swap_bridge.py` | Insert the v1 single-part `A1_UART-Bridge` composite at the bridge pose |
| `mates.py` | Replace Fix/Groups with fastened mates (one fixed ground part per assembly); `--chassis` mates the housing to the plate |
| `check_lidar.py` | A1 (no kit board/stands) on 20+5 standoffs: hole/pad match, clearance, interference -> `T_a1_L.npy` |
| `lidar_swap.py` | Lidar as v1's `RPlidar A1 Base`/`Head` sub-assemblies, mated (its home-made standoffs and the head are gone in v2) |
| `bridge_spacers.py` | Bridge on v1's 4 × `m1.5 rubber spacer` (0.5 mm) instead of 5 mm standoffs: moves its mate connector, inserts + mates the spacers |
| `standoff_info.py` | Origin, length and stud of v1's Bossard standoff Part Studios (1 call each) |
| `check_board.py` | v2 board mount: raised PCB vs base bosses and the 6 board standoffs |
| `board_bosses.py` | v2: Part Studio features on the imported base: floor bosses, M2.5 insert holes, plate screw clearance (7 calls) |
| `board_standoffs.py` | v2: raise offset mates by `RAISE`, mate the new cover and 18 doc standoffs (~45 calls) |
| `check_v2.py` | v2 standoffs + cover pose errors (1 call) |
| `robot_head.py` | Lidar head in `robot_assm` (top level): `dof_lidar` revolute to the housing's lidar base (3 calls) |
| `hw_survey.py`, `hw_heights.py` | Local: plate holes/thickness under the housing screws, bridge holes; screw seat heights in L |
| `hw_frames.py` | Part-frame bounding boxes of the hardware parts (1 call each) |
| `hw_inserts_ps.py` | Native `heat-set inserts` Part Studio, M2/M2.5/M3 (5 calls) |
| `hw_mates.py` | PCB on its standoff + all housing hardware; `--chassis` base → plate screws/nuts. 1 call per mate, implicit connectors |
| `check_hw.py` | Hardware pose errors (1 call; `--chassis`) |
| `verify.py` | Every placed leaf (and the chassis copy) vs intended transform; expect 0.0000 mm (3 calls) |
| `inspect_asm.py`, `plate*.py`, `boardholes.py` | STEP hierarchy, chassis plate and board hole maps |
| `reg.py`, `meshlib.py`, `bb.py`, `_env.py` | Shared helpers |

Full ICP of the whole Orin model is too slow (>2 h); use hole-pattern alignment (`orin_pose.py`).

Workflow lessons (API thrift, transforms, mates, Chrome): `docs/ai-cad-workflow.md` §8–12.
