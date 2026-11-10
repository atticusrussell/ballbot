import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Local hardware survey (no calls): plate holes under the housing screws, plate thickness, bridge hole size."""
import sys, numpy as np
from build123d import import_step, Solid
from OCP.BRepTools import BRepTools
from OCP.TopoDS import TopoDS_Shape
from OCP.BRep import BRep_Builder
from boardholes_lib import holes
sys.path.insert(0, "/home/atticus/workspace/ballbot/src/ballbot_description/cad/parts")
from orin_housing import PLATE_SCREWS, COVER_SCREWS
from place import CHASSIS_L
sh = TopoDS_Shape(); BRepTools.Read_s(sh, "top_plate.brep", BRep_Builder()); plate = Solid(sh)
bb = plate.bounding_box(); print("plate z", round(bb.min.Z, 2), round(bb.max.Z, 2), "t", round(bb.max.Z - bb.min.Z, 2))
H = holes(plate)
for x, y in PLATE_SCREWS:  # frame L -> chassis: + CHASSIS_L
    px, py = x + CHASSIS_L[0], y + CHASSIS_L[1]
    d, hx, hy = min(H, key=lambda h: (h[1] - px) ** 2 + (h[2] - py) ** 2)
    print(f"plate screw L({x}, {y}) -> nearest plate hole Ø{d} off {np.hypot(hx - px, hy - py):.2f} mm")
br = import_step("a1_bridge.step")
print("bridge holes:", [h for s in br.solids() for h in holes(s) if h[0] < 4][:8])
