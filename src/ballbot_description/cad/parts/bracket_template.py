"""Bracket designed against context exported from Onshape (assembly coords, mm).

Run from anywhere after exporting context/motor.step:
  python parts/bracket_template.py
Builds in assembly coordinates (so checks are real), then exports with the
part origin at the datum: hole 0's center on the mating face, +Z along the
motor face's outward normal (Onshape fastened mates make connector frames
identical). Mate a PART_ORIGIN connector to a CENTER connector on that hole.
"""
from pathlib import Path

from build123d import *
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.TopLoc import TopLoc_Location

CAD = Path(__file__).resolve().parents[1]
CONTEXT = CAD / "context/motor.step"
OUT = CAD / "out/motor_bracket.step"

OWNER_POINT = (0, -20, 12)  # any point inside the part with the holes (assembly mm)
HOLE_R = (1.0, 6.0)  # mm; radius range treated as mounting holes (M3 ~ 1.6)
MOUNT = "bottom"     # face of the holed part to sit on: "top" | "bottom"
T = 5.0              # plate thickness
MARGIN = 4.0         # material around each hole
CLEAR = 0.2          # radial clearance over the context hole


def cyl_radius(face: Face) -> float:
    # Face.radius is None for STEP-imported (trimmed) cylinders
    return BRepAdaptor_Surface(face.wrapped).Cylinder().Radius()


def bake(solid: Solid, loc: Location) -> Solid:
    # apply loc to the geometry with no placement left, or export_step nests it
    trsf = (loc * solid.location).wrapped.Transformation()
    s = solid.wrapped.Located(TopLoc_Location())
    return Solid(BRepBuilderAPI_Transform(s, trsf, True).Shape())


# 1. Load context, minus this bracket's previous push (export includes it once mated)
prev_vol = import_step(str(OUT)).volume if OUT.exists() else None
solids = [s for s in import_step(str(CONTEXT)).solids()
          if not (prev_vol and abs(s.volume - prev_vol) < 1e-4 * prev_vol)]
ctx = Compound(solids)
owner = next((s for s in solids if s.is_inside(Vector(OWNER_POINT))), None)
assert owner, f"no context solid contains OWNER_POINT {OWNER_POINT}"

# 2. Mounting holes on that part (CenterOf.MASS = true axis center)
holes = [f for f in owner.faces()
         if f.geom_type == GeomType.CYLINDER and HOLE_R[0] <= cyl_radius(f) <= HOLE_R[1]]
assert len(holes) >= 2, f"need >=2 holes in radius range {HOLE_R}, found {len(holes)}"
for h in holes:
    assert abs(abs(h.axis_of_rotation.direction.Z) - 1) < 1e-6, "template assumes Z-axis holes"
holes.sort(key=lambda f: (round(f.center(CenterOf.MASS).Y, 3), round(f.center(CenterOf.MASS).X, 3)))
pts = [(h.center(CenterOf.MASS).X, h.center(CenterOf.MASS).Y, cyl_radius(h)) for h in holes]
for x, y, r in pts:
    print(f"hole  x={x:8.3f}  y={y:8.3f}  r={r:.3f}")

# 3. Mating face along the hole axis, on the part that owns the holes
bb = owner.bounding_box()
z_face, sign = (bb.max.Z, 1) if MOUNT == "top" else (bb.min.Z, -1)

# 4. Plate covering all holes, on the mating face
rmax = max(r for *_, r in pts)
xs, ys = [p[0] for p in pts], [p[1] for p in pts]
pad = 2 * (rmax + CLEAR + MARGIN)
w, d = max(xs) - min(xs) + pad, max(ys) - min(ys) + pad
cx, cy, cz = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, z_face + sign * T / 2
bracket = Pos(cx, cy, cz) * Box(w, d, T)
for x, y, r in pts:
    bracket -= Pos(x, y, cz) * Cylinder(r + CLEAR, T * 2)

# 5. Checks: real interference with everything in context, and actual contact
overlap = bracket.intersect(ctx)  # None, a Shape, or a ShapeList (list subclass)
overlap = [] if overlap is None else overlap if isinstance(overlap, list) else [overlap]
vol = sum(s.volume for s in overlap)
gap = bracket.distance_to(owner)
print(f"interference {vol:.4f} mm^3 | gap to part {gap:.4f} mm | bracket {bracket.volume:.1f} mm^3")
assert vol < 1e-6, f"bracket collides with context ({vol:.3f} mm^3); try MOUNT='top'/'bottom'"
assert gap < 1e-6, "bracket doesn't touch the mating face"

# 6. Re-origin at the datum: hole 0 center on mating face, +Z = motor face normal
x0, y0, _ = pts[0]
datum = Location((x0, y0, z_face), (0, 0, 0) if MOUNT == "top" else (180, 0, 0))
print(f"datum (assembly coords, mm): ({x0:.3f}, {y0:.3f}, {z_face:.3f})  mount={MOUNT}")
OUT.parent.mkdir(parents=True, exist_ok=True)
part = bake(bracket.solid(), datum.inverse())  # one flat PRODUCT
part.label = OUT.stem  # part name in Onshape; keep stable or re-imports orphan instances
export_step(part, str(OUT))
print(f"wrote {OUT}")
