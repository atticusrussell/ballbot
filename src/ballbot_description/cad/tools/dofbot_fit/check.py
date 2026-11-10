import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import sys, numpy as np, time
from build123d import *
from build123d.topology import Shape
from OCP.gp import gp_Trsf, gp_Mat, gp_XYZ
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.TopLoc import TopLoc_Location
from OCP.BRepAdaptor import BRepAdaptor_Surface
O = "/home/atticus/workspace/ballbot/src/ballbot_description/cad/out/"
t_shell = np.array([86.35373997486488, 124.82484742903172, -2.40665933839182])
H2L = np.eye(4); H2L[:3, 3] = -(t_shell - np.array([0, 0, 1.0]))   # housing -> L

def xf(shape, Mx):
    tr = gp_Trsf(); R = Mx[:3, :3]; t = Mx[:3, 3]
    tr.SetValues(*R[0], t[0], *R[1], t[1], *R[2], t[2])
    return Solid(BRepBuilderAPI_Transform(shape.wrapped.Located(TopLoc_Location()), tr * shape.location.wrapped.Transformation(), True).Shape())

def solids_of(c): return [s for s in c.solids()]

def housing_part(label):  # v2 cover / base (out/orin_cover.step, out/orin_base.step) by label
    stem = {"orin_housing_cover": "orin_cover", "orin_housing_base": "orin_base"}[label]
    return next(s for s in import_step(O + stem + ".step").children if s.label == label)

def ivol(a, b):
    if not a.bounding_box().overlaps(b.bounding_box(), 0.0) if hasattr(a.bounding_box(), "overlaps") else False: return 0.0
    r = a.intersect(b)
    r = [] if r is None else r if isinstance(r, list) else [r]
    return sum(s.volume for s in r)

def bbox_hit(a, b):
    A, B = a.bounding_box(), b.bounding_box()
    return not (A.max.X < B.min.X or B.max.X < A.min.X or A.max.Y < B.min.Y or B.max.Y < A.min.Y or A.max.Z < B.min.Z or B.max.Z < A.min.Z)

def clash(name, part, others, tol=1e-3):
    tot, worst = 0.0, []
    for lbl, s in others:
        if not bbox_hit(part, s): continue
        r = part.intersect(s); r = [] if r is None else r if isinstance(r, list) else [r]
        v = sum(x.volume for x in r)
        if v > tol: tot += v; worst.append((round(v, 2), lbl))
    worst.sort(reverse=True)
    print(f"{name}: interference {tot:.2f} mm3", worst[:6])

if __name__ == '__main__':
    t0 = time.time()
    cover, base = housing_part("orin_housing_cover"), housing_part("orin_housing_base")
    hous = import_step("housing.step")
    pcb = next(c for c in hous.children if c.label.startswith("YB-EVV01"))
    Dp = H2L @ np.load("D_pcb_snapped.npy")
    pcb_L = [(s.label or "pcb", xf(s, Dp)) for s in pcb.solids()]
    orin = import_step("orin_assm.step")
    Do = H2L @ np.load("D_orin.npy")
    orin_L = [(s.label or "orin", xf(s, Do)) for s in orin.solids()]
    print("loaded", round(time.time() - t0), "s;", len(pcb_L), "pcb solids,", len(orin_L), "orin solids")

    # Orin carrier holes vs PCB Ø2.6 (Jetson 86x58) holes, in L
    car = next(c for c in next(c for c in orin.children if c.label.startswith("600-")).children if c.label.startswith("600-"))
    oh = np.array([(0, 0, 0), (86, 0, 0), (0, 58, 0), (86, 58, 0)], float)
    ohL = (Do @ np.c_[oh, np.ones(4)].T).T[:, :3]
    ph = np.array([(220.03, 173.0), (220.03, 231.0), (306.03, 173.0), (306.03, 231.0)])
    phL = (Dp @ np.c_[ph, np.full(4, -129.84), np.ones(4)].T).T[:, :3]
    d = [np.min(np.linalg.norm(phL[:, :2] - p[:2], axis=1)) for p in ohL]
    print("Orin holes L:", np.round(ohL, 2).tolist()); print("-> nearest PCB 86x58 hole XY error:", np.round(d, 2).tolist(),
          "| gap PCB top->carrier bottom:", round(ohL[0, 2] - phL[0, 2], 2))
    np.save("orin_holes_L.npy", ohL)


    clash("cover x PCB", cover, pcb_L); clash("base x PCB", base, pcb_L)
    clash("cover x Orin", cover, orin_L); clash("base x Orin", base, orin_L)
    print("top inner clearance over Orin:", round(75.0 - max(s.bounding_box().max.Z for _, s in orin_L), 2), "mm")
