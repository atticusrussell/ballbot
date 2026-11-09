import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
from build123d import *
from OCP.BRepAdaptor import BRepAdaptor_Surface
from collections import Counter
import numpy as np
c = import_step("chassis.step")
pa = next(x for x in c.children if x.label.startswith("plate_assm"))
top = max((x for x in pa.children if x.label == "chassis_plate"), key=lambda s: s.bounding_box().min.Z)
from OCP.BRepTools import BRepTools; BRepTools.Write_s(top.wrapped, "top_plate.brep")
holes = []
for f in top.faces():
    if f.geom_type == GeomType.CYLINDER:
        cy = BRepAdaptor_Surface(f.wrapped).Cylinder(); a = cy.Axis(); l = a.Location()
        if abs(abs(a.Direction().Z()) - 1) < 1e-6:
            holes.append((round(cy.Radius() * 2, 2), round(l.X(), 2), round(l.Y(), 2)))
holes = sorted(set(holes))
print("hole dia counts:", Counter(h[0] for h in holes))
for d in sorted(set(h[0] for h in holes)):
    pts = [(h[1], h[2]) for h in holes if h[0] == d]
    print(f"\nØ{d}: {len(pts)}")
    print(" ", sorted(pts))
# outline / non-cyl cutouts: count faces & other edges
print("\nslots/other faces:", Counter(f.geom_type for f in top.faces()))
