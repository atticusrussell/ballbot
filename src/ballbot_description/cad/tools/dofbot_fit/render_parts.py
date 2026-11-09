import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import sys, numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import import_step
from meshlib import tris
O = "/home/atticus/workspace/ballbot/src/ballbot_description/cad/out/"
parts = [(n, tris(import_step(O + n + ".step").wrapped, 0.3)[0]) for n in ("orin_housing_cover", "orin_housing_base")]
fig = plt.figure(figsize=(18, 12))
views = [(parts[0], 25, -60, "cover iso"), (parts[0], -30, -120, "cover from below"), (parts[0], 90, -90, "cover top"),
         (parts[1], 35, -60, "base iso"), (parts[1], 90, -90, "base top"), (parts[0], 0, 0, "cover end (look -X)")]
for i, ((name, T), el, az, ttl) in enumerate(views):
    ax = fig.add_subplot(2, 3, i + 1, projection="3d")
    nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); nrm /= np.linalg.norm(nrm, axis=1)[:, None] + 1e-12
    light = np.clip(0.35 + 0.65 * np.abs(nrm @ np.array([0.3, -0.5, 0.8])), 0, 1)
    ax.add_collection3d(Poly3DCollection(T, facecolor=plt.cm.Blues(0.35 + 0.6 * light), edgecolor="none"))
    mn, mx = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0); c = (mn + mx) / 2; r = (mx - mn).max() / 2
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.view_init(el, az); ax.set_title(ttl); ax.set_axis_off()
plt.tight_layout(); plt.savefig("parts.png", dpi=60)
