"""Printable Orin/YB-EVV01 clamshell housing for the BallBot chassis top plate.

Frame L (mm): origin on the chassis top plate under the housing centre, Z up,
X = shell X (chassis forward), same XY as the DOFBOT shell's zup frame.
z_L = z_shell_zup + 1 (1 mm extra under the PCB vs the DOFBOT stack).

Outputs out/orin_housing.step: cover, base and standoffs as labelled solids, origin = frame L.
"""
from pathlib import Path
import math, sys
from build123d import *
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.TopLoc import TopLoc_Location

CAD = Path(__file__).resolve().parents[1]
OUT = CAD / "out"
ORIG_SHELL = Path.home() / "Downloads/cad/dofbot-pro/extracted/orin_enclosure_zup.step"
DZ = 1.0  # L = zup + DZ

# --- envelope -------------------------------------------------------------
HALF_L = 66.5        # X half length (133 long; PCB is 130)
W_OUT = 58.0         # Y half width outer (116; PCB is 100)
T = 3.0              # wall / top / floor thickness
Z_FLOOR = 3.0        # floor top
Z_TOP = 78.0         # cover outer top
Y_TOPFLAT = 37.5     # half width of flat top; 45 deg chamfers to the walls
GAP = 0.2            # base blocks to cover inner wall

# --- PCB (YB-EVV01, 130 x 100, centred) -----------------------------------
PCB_HOLES = [(sx * 63.35, sy * 48.35) for sx in (-1, 1) for sy in (-1, 1)]
PCB_Z = 10.0         # board bottom
PCB_T = 0.4
ORIN_HOLES = [(x, y) for x in (-48.3, 37.7) for y in (-32.97, 25.03)]  # PCB 86x58 holes (registered)
ORIN_STANDOFF = 20.0  # M2.5 x 20, PCB top -> carrier bottom
PIN_RELIEF = [(56.4, 56.8, -44.6, -44.0), (-54.1, -54.0, 44.0, 47.9), (-57.8, -57.2, 44.4, 47.0),
              (-63.9, -63.5, 44.6, 45.3)]  # PCB bottom-side THT pins over corner blocks (x0, x1, y0, y1)
PIN_CLEAR, PIN_Z = 0.6, 6.5  # pocket margin, pocket floor
# --- inserts (heat-set) ---------------------------------------------------
M3_INS_D, M3_INS_L = 4.0, 6.0
M25_INS_D, M25_INS_L = 3.6, 5.0
M2_INS_D, M2_INS_L = 3.2, 4.0
# --- chassis plate (housing centred at plate X -38.98, Y 0.39) ------------
PLATE_SCREWS = [(-52.5, -43), (-52.5, 43), (5.5, -43), (5.5, 43), (33.5, -29), (33.5, 29)]  # M3, plate Ø2.6
CABLE_HOLES = [(-19.0, 0.0), (26.0, 0.0)]  # over plate Ø25 pass-throughs
COVER_SCREWS = [(sx * 57.5, sy) for sx in (-1, 1) for sy in (-1, 1)]  # x, side; z below
COVER_SCREW_Z = 6.5
# --- lidar (RPLIDAR A1, motor end to -X) ----------------------------------
SCAN = (-17.0, 0.0)
KIT_STACK = 24.6   # A1 dev kit: 15 stand + 1.6 devboard + 8 stand, flange -> pad
LIDAR_PAD = 3.0    # printed pads under the stands; kit clears cover by 5.0
LIDAR_SCREW_D = 2.8  # M2.5 x 10 from inside the cover into the 8 mm stand
LIDAR_HOLES = [(SCAN[0] + 28, SCAN[1] + s * 28) for s in (-1, 1)] + \
              [(SCAN[0] - 42, SCAN[1] + s * 20) for s in (-1, 1)]
# --- A1 UART bridge (35.5 x 20, holes 19 x 14.25), under the lidar lid -----
BRIDGE_C = (42.0, 0.0)  # clear of the A1 footprint (x < 19)
BRIDGE_HOLES = [(BRIDGE_C[0] + dx, BRIDGE_C[1] + dy) for dx in (-7.125, 7.125) for dy in (-9.5, 9.5)]
BRIDGE_STANDOFF = 5.0
# --- fan grille -----------------------------------------------------------
FAN_C = tuple(float(v) for v in sys.argv[1:3]) if len(sys.argv) > 2 else (-4.3, 9.0)
FAN_R = 19.0


def cover() -> Part:
    yc = W_OUT - (Z_TOP - (W_OUT - Y_TOPFLAT))  # unused; keep math explicit below
    z_ch = Z_TOP - (W_OUT - Y_TOPFLAT)          # chamfer start on outer wall
    outer = [(-W_OUT, Z_FLOOR), (W_OUT, Z_FLOOR), (W_OUT, z_ch), (Y_TOPFLAT, Z_TOP),
             (-Y_TOPFLAT, Z_TOP), (-W_OUT, z_ch)]
    wi, zi = W_OUT - T, Z_TOP - T
    k = W_OUT + z_ch - T * math.sqrt(2)          # inner chamfer line y + z = k
    inner = [(-wi, 0), (wi, 0), (wi, k - wi), (k - zi, zi), (-(k - zi), zi), (-wi, k - wi)]
    body = extrude(Plane.YZ * Polygon(*outer, align=None), amount=HALF_L, both=True)
    body -= extrude(Plane.YZ * Polygon(*inner, align=None), amount=HALF_L + 5, both=True)

    # port windows copied from the DOFBOT shell walls (shifted DZ)
    shell = import_step(str(ORIG_SHELL))
    for f in shell.faces():
        n = f.normal_at()
        if f.geom_type == GeomType.PLANE and abs(abs(n.Y) - 1) < 1e-3 and abs(f.center().Y) > 53:
            for w in f.inner_wires():
                hole = Face(w)
                if hole.area > 30:
                    body -= Pos(0, 0, DZ) * extrude(hole, amount=8, dir=(0, 1, 0), both=True)

    # chamfer vent slots
    zc = (z_ch + Z_TOP) / 2; ym = (W_OUT + Y_TOPFLAT) / 2
    for x in range(-54, 55, 9):
        for s in (-1, 1):
            body -= Pos(x, s * ym, zc) * Rot(-45 * s, 0, 0) * Box(3, 16, 12)

    # fan grille: parallel slots clipped to a circle
    grille = Part()
    for i in range(-4, 5):
        yy = FAN_C[1] + i * 4.5
        half = math.sqrt(max(FAN_R ** 2 - (i * 4.5) ** 2, 0))
        if half > 4:
            grille += Pos(FAN_C[0], yy, Z_TOP - T / 2) * Box(2 * half, 2.5, T + 2)
    body -= grille

    # cover -> base screws through the side walls
    for x, s in COVER_SCREWS:
        body -= Pos(x, s * (W_OUT - T / 2), COVER_SCREW_Z) * Rot(90, 0, 0) * Cylinder(1.7, T + 2)

    # lidar pads; kit stands screw down from inside the cover
    for x, y in LIDAR_HOLES:
        body += Pos(x, y, Z_TOP + LIDAR_PAD / 2) * Cylinder(4.5, LIDAR_PAD)
        body -= Pos(x, y, Z_TOP) * Cylinder(LIDAR_SCREW_D / 2, 2 * (LIDAR_PAD + T) + 1)

    # inner bosses under the top for bridge (M2) inserts
    for (x, y), d_boss, d_ins, l_ins in [(p, 6.0, M2_INS_D, M2_INS_L) for p in BRIDGE_HOLES]:
        h = l_ins + 1.5 - T
        body += Pos(x, y, Z_TOP - T - h / 2) * Cylinder(d_boss / 2, h)
        body -= Pos(x, y, Z_TOP - l_ins / 2) * Cylinder(d_ins / 2, l_ins + 0.01)
    return body


def base() -> Part:
    body = Pos(0, 0, Z_FLOOR / 2) * Box(2 * HALF_L, 2 * W_OUT, Z_FLOOR)
    bx0, by0, by1 = 54.0, 44.0, W_OUT - T - GAP
    for sx in (-1, 1):
        for sy in (-1, 1):
            blk = Pos(sx * (bx0 + HALF_L) / 2, sy * (by0 + by1) / 2, (Z_FLOOR + PCB_Z) / 2) * \
                  Box(HALF_L - bx0, by1 - by0, PCB_Z - Z_FLOOR)
            body += blk
    for x0, x1, y0, y1 in PIN_RELIEF:
        w, d = x1 - x0 + 2 * PIN_CLEAR, y1 - y0 + 2 * PIN_CLEAR
        body -= Pos((x0 + x1) / 2, (y0 + y1) / 2, (PIN_Z + PCB_Z) / 2 + 0.5) * Box(w, d, PCB_Z - PIN_Z + 1)
    for x, y in PCB_HOLES:  # vertical M3 inserts for the PCB
        body -= Pos(x, y, PCB_Z - M3_INS_L / 2) * Cylinder(M3_INS_D / 2, M3_INS_L + 0.01)
    for x, s in COVER_SCREWS:  # horizontal M3 inserts for the cover
        y0 = s * (W_OUT - T - GAP)
        body -= Pos(x, y0 - s * M3_INS_L / 2, COVER_SCREW_Z) * Rot(90, 0, 0) * Cylinder(M3_INS_D / 2, M3_INS_L + 0.01)
    for x, y in PLATE_SCREWS:  # M3 button heads, counterbored
        body -= Pos(x, y, Z_FLOOR / 2) * Cylinder(1.7, Z_FLOOR + 1)
        body -= Pos(x, y, Z_FLOOR - 1) * Cylinder(3.25, 2.01)
    for x, y in CABLE_HOLES:
        body -= Pos(x, y, Z_FLOOR / 2) * Cylinder(13.0, Z_FLOOR + 1)
    return body


def bake(s: Solid) -> Solid:  # apply location to geometry; Onshape drops nested STEP placements
    trsf = s.location.wrapped.Transformation()
    return Solid(BRepBuilderAPI_Transform(s.wrapped.Located(TopLoc_Location()), trsf, True).Shape())


def hex_standoff(af: float, length: float) -> Part:
    return extrude(RegularPolygon(af / math.sqrt(3), 6), amount=length)


def hardware() -> list[tuple[str, Part]]:
    z_pcb = PCB_Z + PCB_T
    sets = [("standoff_m2x5_bridge", BRIDGE_HOLES, Z_TOP, 4.0, BRIDGE_STANDOFF),
            ("standoff_m25x20_orin", ORIN_HOLES, z_pcb, 5.0, ORIN_STANDOFF)]
    return [(f"{name}_{i + 1}", Pos(x, y, z) * hex_standoff(af, h))
            for name, holes, z, af, h in sets for i, (x, y) in enumerate(holes)]


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    parts = [("orin_housing_cover", cover()), ("orin_housing_base", base())] + hardware()
    solids = []
    for name, p in parts:
        assert len(p.solids()) == 1, name
        s = bake(p.solids()[0]); s.label = name; solids.append(s)  # geometry in place, no STEP transforms
        if not name.startswith("standoff"):
            print(f"{name}: vol={p.volume:.0f} mm3 bbox={p.bounding_box()}")
    asm = Compound(children=solids, label="orin_housing")  # names/structure fixed after 1st push
    export_step(asm, str(OUT / "orin_housing.step"))
    print(f"wrote {OUT / 'orin_housing.step'} ({len(solids)} solids)")
