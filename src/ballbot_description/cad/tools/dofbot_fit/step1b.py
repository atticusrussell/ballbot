import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import numpy as np
from build123d import import_step
from scipy.spatial import cKDTree
from reg import rot24, H
X = "/home/atticus/Downloads/cad/dofbot-pro/extracted/"
fv = np.array([tuple(v) for v in import_step(X + "orin_enclosure_assembly-frame.step").vertices()])
shell = next(c for c in import_step("housing.step").children if c.label == "orin_enclosure")
hv = np.array([tuple(v) for v in shell.vertices()])
print(len(fv), len(hv))
tree = cKDTree(hv); res = []
for R in rot24():
    p = fv @ R.T; t = (hv.min(0) + hv.max(0)) / 2 - (p.min(0) + p.max(0)) / 2
    d, _ = tree.query(p + t); res.append((d.max(), d.mean(), R, t))
res.sort(key=lambda r: r[0])
for r in res[:4]: print("max", round(r[0], 4), "mean", round(r[1], 4), r[2].tolist(), np.round(r[3], 3))
M = H(res[0][2], res[0][3]); np.save("M_fused_to_housing.npy", M); print(np.round(M, 4))
