# Maps

Maps live here as `<name>.pgm` + `<name>.yaml` and are loaded by
`ballbot_navigation/launch/navigation.launch.py` via the `map:=` argument.
`MAP_NAME` in that file sets the default.

| Map | Space | Captured | Resolution | Extent |
|---|---|---|---|---|
| `bluebrook` | Bluebrook house, first floor (single storey) | 2026-08-11 | 0.05 m/px | 338 × 293 px, 16.9 × 14.7 m |

## `bluebrook`

First map built on a correct TF chain. Captured with `slam_toolbox` in
`online_async` mode, driven by teleop.

Manually cleaned up in GIMP after capture:

- The lidar sees **through the glass door**, so it mapped free space outside the
  house. The door line was painted occupied and the space beyond it unknown.
- Chair legs around the island and dining table were erased. They move, and stale obstacles in
  the static map make the planner route around empty floor.

This only stops the *global* planner leaving through the door. The local costmap
is built from live scans and the lidar will never see that glass, so it is not a
collision guarantee — a `nav2_costmap_2d` keepout filter is the real fix.

Known defects:

- Room boundaries are closed and square, but coverage is not exhaustive; some
  edges are single-pass.
