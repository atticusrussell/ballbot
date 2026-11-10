"""RPLIDAR A1 mount: M2.5 hex standoffs (20 M-F + 5 M-F per hole) on the cover's lidar pads, frame L.

Replaces the A1 dev-kit stands/devboard. M2.5 x 10 screws up from inside the cover into the 20 mm standoffs.
Outputs out/lidar_standoffs.step (separate import so the orin_housing parts and their mates stay untouched).
"""
from build123d import Compound, Pos, export_step
from orin_housing import OUT, Z_TOP, LIDAR_PAD, LIDAR_HOLES, bake, hex_standoff, BRASS

STACK = [("standoff_m25x20_lidar", 20.0), ("standoff_m25x5_lidar", 5.0)]  # bottom -> top
LIDAR_STACK = sum(h for _, h in STACK)  # pad top -> A1 mount face; A1 lowest point clears the cover by ~5.5


def standoffs():
    out, z = [], Z_TOP + LIDAR_PAD
    for name, h in STACK:
        out += [(f"{name}_{i + 1}", Pos(x, y, z) * hex_standoff(5.0, h)) for i, (x, y) in enumerate(LIDAR_HOLES)]
        z += h
    return out


if __name__ == "__main__":
    solids = []
    for name, p in standoffs():
        s = bake(p.solids()[0]); s.label = name; s.color = BRASS; solids.append(s)
    export_step(Compound(children=solids, label="lidar_standoffs"), str(OUT / "lidar_standoffs.step"))
    print(f"wrote {OUT / 'lidar_standoffs.step'} ({len(solids)} solids), stack {LIDAR_STACK} mm")
