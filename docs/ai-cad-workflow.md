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
