import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Orin pose from hole alignment (86x58 on PCB, 20 mm gap); pick 0/180 deg by fused-point overlap."""
import numpy as np
from build123d import import_step
from scipy.spatial import cKDTree
GAP = 20.0  # M2.5x20 standoffs, PCB top -> carrier bottom (fused measurement)
Dp = np.load("D_pcb_snapped.npy")
ph = np.array([(220.03, 173.0), (220.03, 231.0), (306.03, 173.0), (306.03, 231.0)])
phH = (Dp @ np.c_[ph, np.full(4, -129.84), np.ones(4)].T).T[:, :3]  # housing frame, PCB top
o = import_step("orin_assm.step")
car = next(c for c in next(c for c in o.children if c.label.startswith("600-")).children if c.label.startswith("600-"))
cb = car.bounding_box(); print("carrier bbox", cb)
ctr_o = np.array([43.0, 29.0, cb.min.Z])  # hole-pattern centre on carrier bottom
ctr_h = np.r_[phH[:, :2].mean(0), phH[0, 2] + GAP]
tgt = np.load("tgt_housing.npy"); tree = cKDTree(tgt)
kids = [c for c in o.solids()]
best = None
for a in (0, 90, 180, 270):
    c, s = np.cos(np.radians(a)), np.sin(np.radians(a)); R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    D = np.eye(4); D[:3, :3] = R; D[:3, 3] = ctr_h - R @ ctr_o
    hol = (D @ np.c_[[(0, 0, cb.min.Z), (86, 0, cb.min.Z), (0, 58, cb.min.Z), (86, 58, cb.min.Z)], np.ones(4)].T).T[:, :3]
    err = max(np.min(np.linalg.norm(phH[:, :2] - p[:2], axis=1)) for p in hol)
    # sample solid vertices -> nearest fused point distance
    v = np.array([tuple(x) for k in kids for x in k.vertices()][::3]); vh = (R @ v.T).T + D[:3, 3]
    d, _ = tree.query(vh); score = (d < 1.0).mean()
    print(f"rot {a:3d}: hole err {err:.2f} mm | vertex inlier {score:.3f} median {np.median(d):.2f}")
    if err < 0.5 and (best is None or score > best[0]): best = (score, D)
print(np.round(best[1], 4)); np.save("D_orin.npy", best[1])
