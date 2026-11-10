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
| `fix_kit.py` | Full A1 dev-kit stack (devboard, connector, 15/8 mm stands) at kit positions on the cover pads |
| `swap_bridge.py` | Insert the v1 single-part `A1_UART-Bridge` composite at the bridge pose |
| `mates.py` | Replace Fix/Groups with fastened mates (one fixed ground part per assembly); `--chassis` mates the housing to the plate |
| `verify.py` | Every placed leaf (and the chassis copy) vs intended transform; expect 0.0000 mm (3 calls) |
| `inspect_asm.py`, `plate*.py`, `boardholes.py` | STEP hierarchy, chassis plate and board hole maps |
| `reg.py`, `meshlib.py`, `bb.py`, `_env.py` | Shared helpers |

Full ICP of the whole Orin model is too slow (>2 h); use hole-pattern alignment (`orin_pose.py`).

Workflow lessons (API thrift, transforms, mates, Chrome): `docs/ai-cad-workflow.md` §8–12.
