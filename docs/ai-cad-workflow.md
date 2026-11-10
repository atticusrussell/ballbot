# BallBot AI CAD workflow (Onshape + Claude Code + code-CAD)

How Claude Code works on the BallBot Onshape model: what it can change, where FeatureScript and code-based CAD
fit in, and the loops for designing brackets and mounts against existing parts.

Validated end to end on the scratch doc on 2026-11-08 (build123d 0.12.0, Python 3.10, Onshape library 3112).


## 1. Who does what

| Task | Tool | Why |
|---|---|---|
| Brackets/mounts tied to parts in the **same Part Studio** | FeatureScript MCP (Onshape Labs) | Parametric custom feature; rebuilds when the referenced holes/faces move |
| Reusable generators (hole patterns, gussets, pulleys) | FeatureScript MCP | One feature, applied many times, no Claude needed after |
| Parts designed against things **across the assembly**, or complex parts | Code-CAD loop (build123d → STEP → Onshape) | Python + numeric checks; diffable in git; no live link |
| Change dimensions of existing parts | onshape-mcp `set_variable` | Reliable if the part is driven by variables/configs |
| Place parts in assemblies, exports | onshape-mcp | Insert, transform, screenshots, positions, STEP/STL/GLTF export |
| Mate code-CAD parts | `onshape_step_sync.py mate` | onshape-mcp can't build part-origin or hole-center connectors |
| Tightly coupled in-context parts (e.g. around the DOFBOT arm) | Manual in Onshape | Needs a live in-context link across Part Studios |

Rule of thumb:

- Same Part Studio as the thing it attaches to → FeatureScript.
- Anything else → code loop.
- Assembly placement either way → onshape-mcp + the sync script's `mate`.


## 2. API budget (read this first)

- All three paths spend the same Onshape API allocation: API keys (sync script, onshape-mcp) and the FeatureScript
  MCP's OAuth session.
- Current plan: **2,500 requests per year** (cycle 2026-05-20 → 2027-05-20), overage disabled.
  When it runs out, every call fails until the cycle resets.
- Check it with the FeatureScript MCP `get_api_usage` tool.
- Rough costs:
  - one code-loop iteration (export + push + check) ≈ 10–15 calls
  - one FeatureScript debug round ≈ 2–4 calls
  - the full validation session cost ~140
- Keep requests small. Avoid onshape-mcp `get_features` (huge output) and repeated screenshots.


## 3. One-time setup

- Onshape API keys: https://dev-portal.onshape.com/keys. Export `ONSHAPE_ACCESS_KEY` / `ONSHAPE_SECRET_KEY`.
  These keys have full access to your account, so never commit them.
- onshape-mcp: `git clone https://github.com/hedless/onshape-mcp`, `pip install -e .`, add it to Claude Code's MCP
  config with the keys in `env`. Smoke test: "list my Onshape documents".
- FeatureScript MCP: subscribe to "Onshape Labs FeatureScript MCP" in the Onshape App Store, add the HTTP MCP
  server `https://fs-mcp.labs.onshape.app/mcp`, authenticate via `/mcp`.
- CAD Python env, kept **out of the ROS Python** (ROS Humble needs numpy < 2; build123d pulls numpy 2):
  ```
  python3 -m venv ~/.venvs/ballbot-cad
  ~/.venvs/ballbot-cad/bin/pip install build123d requests
  ```
  Run every CAD script with `~/.venvs/ballbot-cad/bin/python`. Don't put the venv inside the repo (colcon crawls it).

Layout:
```
src/ballbot_description/cad/
  tools/onshape_step_sync.py   # elements | export | push | edges | mate
  parts/<part>.py              # build123d sources (committed)
  featurescript/<feature>.fs   # FeatureScript sources (committed; Onshape copy is the live one)
  out/<part>.step              # generated (committed) + .onshape_imports.json
  context/                     # STEP exported from Onshape (gitignored)
```


## 4. FeatureScript loop (same-Part-Studio parts)

1. Read the server's notes first (`read_featurescript_notes`). They hold the lessons from earlier sessions.
2. Get the live library version: `test_featurescript` with `function(context, id) { return "ok"; }`.
3. Write the logic as an exported function, plus a **self-test feature** (empty precondition) that builds its own
   test geometry, calls the function, and `println`s checks (`evBox3d`, `evVolume`, `evCollision`).
   Iterate with `test_feature` until clean. It runs the *first* exported feature in the file.
4. Save the source to `cad/featurescript/<name>.fs`, `create_feature_studio` in the target doc, then
   `put_featurescript` with the production feature only.
5. Insert the feature into the Part Studio (Onshape UI, or REST: `BTMFeature-134`, `featureType` = exported const
   name, `namespace` = `e<studioEid>::m<studioMicroversion>`, query params as deterministic IDs).
6. Get edge/face IDs with `onshape_step_sync.py edges "<part studio URL>"`.

Validated: `featurescript/hole_bracket.fs` built a plate on the scratch block's two bottom hole rims. Then the
small hole was moved 18.2 → 28 mm in the sketch, and the plate rebuilt to follow it exactly (max Y 27.38 → 37.16 mm).


## 5. Code-CAD loop (assembly-context parts)

1. Safety: work in an Onshape branch, or let `push` make restore-point versions (default on updates).
2. Export context in assembly coordinates:
   `python tools/onshape_step_sync.py export "<assembly tab URL>" -o context/chassis.step`
   (or a Part Studio URL + `--part-id <id>`).
3. Design in code (see `parts/bracket_template.py`):
   - `OWNER_POINT` picks the part to design against. Real assemblies have many holed parts.
   - Measure real hole and face positions, then build in the same coordinates.
   - Check real interference with `intersect` against everything in context, and check contact with `distance_to`.
   - Export with the part origin at the datum: a mounting-hole center on the mating face, +Z = the target face's
     outward normal.
4. Push: `python tools/onshape_step_sync.py push "<doc workspace URL>" out/<part>.step`.
   - The first push creates an import tab.
   - Later pushes update the same tab in place (tracked in `out/.onshape_imports.json`).
5. Place (first time only):
   - onshape-mcp `add_assembly_instance`, then read the instance id back from `get_assembly_positions`
     (the add tool prints "unknown").
   - Find the target hole rim: `python tools/onshape_step_sync.py edges "<target part studio URL>"`.
   - Mate:
     `python tools/onshape_step_sync.py mate "<assembly URL>" --origin-of <partInst> --to <targetInst>:<edgeId> --name <part>_datum`
6. Rerun the onshape-to-robot pipeline in `ballbot_description` to regenerate the URDF.
7. Iterate: edit the script (or re-export after upstream changes), rerun, push. The mate and instance survive.

Validated:

- Thickness change 3 → 5 mm: same tab updated, restore point created, mate stayed OK.
- After the upstream hole move: re-export, rerun, push picked up the new hole; mate stayed OK.

Conventions (each one learned the hard way in validation):

- **Mate with `PART_ORIGIN` → `CENTER` connectors** (what `mate` builds).
  - The origin connector references no topology, so re-imports can't break it.
  - onshape-mcp's connectors use `CENTROID` inference, which errors on hole-rim edges.
- **Fastened mates make the two connector frames identical, not opposed.** The hole-rim connector's +Z is the
  target face's outward normal, so the part's datum +Z must match it.
- **Export one flat, labelled `Solid` with no placement.** The template's `bake()` does this.
  - `Location * shape` keeps a placement, and `export_step` writes that as a nested assembly.
  - **Never change the STEP's product name or structure after the first push.** Onshape then re-imports it as a
    different part and the assembly instance loses its part (`partId` goes empty).
- Code parts are plain solids in Onshape. All edits happen in the script. No live link: if neighbors move,
  re-export the context and rerun.
- Once mated, the assembly export contains the part itself. The template skips solids whose volume matches the
  last `out/*.step`.
- Units are mm end to end (verified: 18.32 × 36.54 × 3.00 mm in build123d and in Onshape).


## 6. Tool capabilities and gaps

onshape-mcp can:

- create assemblies; insert, transform and position instances
- create face-based mate connectors and 4 mate types
- read positions, bounds, feature health and face IDs
- run read-only FeatureScript eval
- take screenshots
- export STEP/STL/GLTF

onshape-mcp can't:

- import files (use `push`)
- do real interference checks (bounding boxes only; use build123d `intersect`)
- make origin or edge-center mate connectors (use `mate`)
- sketch on existing geometry (use FeatureScript)
- drawings, BOMs, configuration parameters

FeatureScript MCP:

- Part Studios only: no assemblies, mates, variables or export.
- Early-access Labs product.
- Compile errors come back as a bare "no features found" (see gotchas).


## 7. Gotchas

- Claude doesn't see the model through MCP. Use numeric checks plus screenshots, and screenshots sparingly
  (API budget).
- Assemblies are the weakest area. Keep requests small: one part, one mate.
- Drive Onshape parts with variables so edits are numbers, not geometry.
- build123d 0.12:
  - `Face.radius` is `None` for STEP-imported cylinders. Use
    `BRepAdaptor_Surface(face.wrapped).Cylinder().Radius()`.
  - `face.center(CenterOf.MASS)` gives a hole's axis center. Plain `.center()` lands on the hole wall.
  - `a.intersect(b)` returns `None`, a `Shape`, or a `ShapeList` (multi-solid context). Handle all three.
- FeatureScript:
  - test_feature "could not find Trojan feature spec: no features found" means a compile error. Bisect it.
  - A local named after a std symbol (e.g. `box`) causes this.


## 8. Starting from scratch: what to read and where things are

Learned on the v2 Orin/PCB housing work, 2026-11-09. Onshape version `2.0.2-alpha` on branch `ballbot-v2-ai` is
the verified layout locked with Fix/Groups; the workspace after it replaces those with fastened mates (§10).

- **Onshape doc:** BallBot `833f9762904995f22ff7dd9f`.
  - Work branch `ballbot-v2-ai` (workspace `9dc077545bceb395c1a5f5aa`). Never edit `main` or the v1 versions.
  - `still_chassis_assm` (`69b4604a7be4b0e3dbe3d002`) is the top-level robot.
  - `dofbot_pcb_housing` (`76281463014c048875567be7`) holds the Orin + YB-EVV01 PCB + printed housing + RPLIDAR A1
    kit + A1 UART bridge.
- **Chassis plate hole drawing:** https://cad.onshape.com/documents/d65a817fa3138e6d6ce62340/w/d9ae3e7c456080d5144ab443/e/22d3c4e40567ae2881f8a79b
  It is only a legend of hole groups (A–M), with no sizes or threads. Hole sizes were inferred from STEP (Ø2.6 = M3 tapped?,
  Ø3.1 = M3 clearance); test-fit before printing.
- **Repo layout:**
  - `cad/tools/` holds every utility script, including one-offs. Never leave scripts in a scratchpad; they get lost.
  - `cad/tools/dofbot_fit/` holds the housing-placement scripts; see its README for a per-script table.
  - `cad/context/<task>/` (gitignored) holds STEP exports, point clouds, `.npy` transforms and resumable state
    (`placed.json`). Scripts `import _env` to `chdir` there.
  - `cad/parts/orin_housing.py` is the build123d housing source; `cad/out/orin_housing.step` is its export.
- **Before any Onshape call:** check the remaining budget (§2) and tell the user that MCP/API work spends it.


## 9. API-thrifty working style

- Export context once (`onshape_step_sync.py export`), then do everything you can locally on the STEP:
  registration, hole maps, interference, clearances, renders (matplotlib previews in `dofbot_fit/render*.py`).
- Batch REST work into small resumable scripts that record ids in `cad/context/<task>/placed.json`. A rerun skips
  finished steps, so a crash or a fix never re-inserts things twice.
- Put the call count in each script's docstring (`~9 API calls`) so the cost is known before running it.
- Do UI work (unfix, delete many instances or features, visual checks) in Chrome when it's connected: zero API
  cost (§11).
- **Never "work around" a missing UI by spending hundreds of API calls** (e.g. delete + re-insert parts to clear
  Fix). If Chrome is logged out or disconnected, stop and ask the user to sign in. Claude won't sign in for them,
  even with a password manager.
- When a POST returns something unexpected, check in Chrome (or with one GET) whether it actually happened before
  retrying, so nothing gets inserted twice.
- Typical costs on this task: an insert + transform ≈ 3 calls; `verify.py` = 3 calls; mates ≈ 2–3 each; the whole first placement
  ≈ 30; a botched round with repairs ≈ 75. The two mistakes in §10 cost more than all the planned work.


## 10. Onshape assembly lessons (each one learned the hard way)

**Transforms**
- `POST .../occurrencetransforms` with `isRelative: false` on a **sub-assembly** sets every leaf inside it to that
  one pose: the sub-assembly collapses into a pile. Absolute is only safe on a top-level **part** instance.
- For a sub-assembly: insert it fresh (identity pose) and apply the target as a **relative** transform.
- Never "fix" one leaf of a sub-assembly from the parent; it scrambles the rest. Delete and re-insert instead
  (`repair_pcb.py`).
- Transforms are 4×4 row-major, translations in **metres**. Keep mm locally and convert at the API boundary
  (`place.to_m`).

**Instances and sub-assemblies**
- `GET /assemblies/...` leaves **composite parts** out of `instances`/`occurrences` unless you pass
  `?includeNonSolids=true`. Without it an insert looks like it returned nothing.
- A sub-assembly copy in a parent goes **stale** when the child changes (new instances don't show up in the
  parent's occurrence paths). Fix: delete the copy, re-insert it, apply the relative transform again.
- STEP files with nested placements import with the placements dropped in the parent assembly (standoffs floated
  at the origin). Bake geometry into world position and export one flat assembly, or push as a Part Studio.
- Prefer the existing single-part / composite version of a vendor model (e.g. `A1_UART-Bridge`, partId `JGD`,
  which v1 already uses) over inserting a whole Part Studio. Whole-studio inserts bring in every body as a loose
  instance (111 for the bridge).
- **Check how v1 structures a component before inserting it, and reuse its sub-assemblies / composite parts.**
  v1 already has `RPlidar A1 Base` + `RPlidar A1 Head` assemblies, `A1_UART-Bridge` (composite part) and more.
  Inserting the vendor Part Studio instead (twice, lidar and bridge) produced dozens of loose parts and redo work.
- Vendor sub-assemblies like the lidar Base/Head are laid out in the vendor's original coordinates, the same as
  the `onepart` studio. Check one part's position against a local export, then place the sub-assembly with the
  same transform the studio parts used (relative, on a fresh insert).
- Don't put the vendor dev-kit hardware (devboard, kit stands) into the robot. The lidar sits on M2.5 standoffs
  (20 + 5 mm) generated by `cad/parts/lidar_standoffs.py`. That's a separate STEP import, so the existing
  `orin_housing` import and its mates stay untouched.
- Add new code-CAD parts as a **new** STEP import rather than adding solids to an existing one. Changing an
  imported STEP's structure re-imports its parts and breaks every mate on them.

**Locking things in place: mates, not Fix/Group (the user's preference)**
- Subassemblies must not have floating parts, and a pile of Fixed parts or Groups counts as sloppy. Use exactly
  one fixed ground part per assembly (`orin_housing_base`; v1's `plate assm` in the chassis) and fastened mates
  for everything else. `dofbot_fit/mates.py` does this; follow its pattern.
- Parts inserted from one Part Studio share a part origin. Mate them origin-to-origin with no offset (one shared
  connector on the parent part, one per child).
- Everything else: a mate connector on the parent part's origin, moved by the current relative pose
  (`transform` + `translationX/Y/Z` + `rotationType`/`rotation`), mated to the child's origin connector. The
  connector can only rotate about one axis, so the pose must be Z-only rotation; set `flipPrimary` (180° about X,
  applied before the move) when the child's Z is flipped. Positions survive exactly (verified 0.0000 mm).
- A sub-assembly is mated through one of its leaf **parts** (path `[subInst, partInst]`). The whole rigid
  sub-assembly follows. Pick a leaf whose relative rotation the connector can express.
- **A mate between two Fixed instances errors.** Unfix everything except the ground part first. There's no REST
  call for Fix/Unfix: do it in the UI (right-click → Unfix on click + shift-click ranges).
- Inline implicit connectors (`BTMInferenceQueryWithOccurrence` directly in a mate's `mateConnectorsQuery`) don't
  resolve over the API. Create explicit `BTMMateConnector-66` features and reference them with
  `BTMFeatureQueryWithOccurrence-157`.
- Enum parameters need the exact `enumName` from the spec, e.g. `rotationType` is `"Rotation axis"` with values
  `ABOUT_X/Y/Z`. Read it once from `GET /assemblies/.../featurespecs` (saved locally as
  `context/dofbot_fit/asm_featurespecs.json`) instead of guessing; each wrong guess is a failed call.
- A failed mate feature still gets created (status `ERROR`). Delete it at once
  (`DELETE .../features/featureid/<id>`; `mates.py` does this) so retries don't stack up broken features.
- Remove old Groups before mating their members, or they conflict. **Don't create Groups over the API**: the
  API-created Group rotated everything ~90°.
- Cost: about 2 calls per no-offset mate and 3 per offset mate; 43 mates ≈ 90 calls.

**Verify numerically, every time**
- `dofbot_fit/verify.py` (3 calls) compares every leaf occurrence of every placed instance, including the copy
  inside the parent assembly, against the intended transform. Target: 0.0000 mm. Run it after any change,
  including UI-only changes like grouping.
- Then one visual check (Chrome, free; API screenshots cost calls and are low-res).

**Interference checks (build123d)**
- Full `intersect`/`distance_to` of the detailed cover against every vendor solid runs >25 min. First rule pairs out
  by bounding boxes (e.g. the A1's lowest z above the cover's highest z), then intersect only overlapping pairs
  (`check_lidar.py`). Print with `-u` / `flush=True` and save results before the slow part.

**Registration (placing a vendor model where another model had it)**
- Full ICP on a big model (Orin, 1,883 bodies) is too slow (>2 h). Register on features instead: hole patterns,
  big planar faces, shell vertices over the 24 axis-aligned rotations (`step1b.py`, `orin_pose.py`).
- Snap results to round mm / exact axes once you know they should be (`D_pcb_snapped.npy`).


## 11. Claude in Chrome (use it whenever it's connected)

Onshape in the user's own Chrome session spends **no API calls**. It is the right tool for visual checks, reading
drawings, and UI-only operations (Fix, Group, bulk delete, suppress).

**Setup**
- Chromium only (Chrome, maybe Edge); no Firefox. The user can keep Firefox for their own work and run Chrome
  alongside.
- Install the Claude in Chrome extension, sign in with the same claude.ai account, then `/chrome` in Claude Code
  (or start with `claude --chrome`). If the tools still don't appear, restart the session.
- If WebGL is sluggish on Linux, check `chrome://gpu` and enable hardware acceleration.

**Usage**
- Load all tools in **one** ToolSearch call (`tabs_context_mcp, tabs_create_mcp, navigate, computer, find,
  read_page, get_page_text, tabs_close_mcp`).
- Call `tabs_context_mcp` first, open a **new tab** for the Onshape URL, close it when done.
- Use `find` / `read_page` to locate elements by name (instance list rows, menu items) instead of guessing pixel
  coordinates; take screenshots for the 3D view.
- `get_page_text` reads drawings and the instance list cheaply.
- Never trigger browser dialogs (alerts/confirm). Onshape's own delete confirmations are in-page and fine.

**Onshape UI tricks**
- Instance list multi-select: **click + shift-click** ranges. Ctrl-click is unreliable through the extension.
- Clear a selection from the right-click menu ("Clear selection") before starting a new one.
- Right-click on a selection → Fix / Unfix / Delete / Suppress. Deleting 100+ instances is one UI action vs one
  API call each. A sub-assembly instance in a selection hides Fix/Unfix.
- The instance-list filter box also finds mate features (type "Group" to find Groups to delete).
- Drag the instance panel's right edge to widen it so names are readable in screenshots.
- After UI changes, re-run `verify.py` (UI edits can move things too).

- The user signs in to Onshape in Chrome themselves. If the tab shows "Sign in" / "Create account", stop and ask.

**Recommendation:** script geometry and placement (exact numbers, resumable, reviewable in git), and do
selection-heavy housekeeping plus all visual checks in Chrome. If Chrome isn't connected and a UI task would cost
dozens of API calls, ask the user to connect it (or do it themselves) instead of spending the budget.


## 12. Conventions with the user

- Commit only when asked; push to the working branch (`claude-cad-experiments`), never `humble` directly.
- Onshape versions mark milestones on the branch: `<major>.<minor>.<patch>-alpha` (e.g. `2.0.1-alpha`,
  `2.0.2-alpha`) with a one-line description of the state. Make one before risky bulk changes too.
- Never commit API keys. Never edit `docs/` unless asked; report findings in chat.
- Never edit vendor/source Part Studios (lidar, Orin, PCB); place them as they are.
- Terse one-line code comments. Chat replies: prose wrapped at ~115 chars, lots of bullets.
- Ask before deleting things the user made. Leftovers from your own work (old instances, test parts, failed
  features) get cleaned up.
