import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
T = np.load("region_tris.npy"); fid = np.load("region_fid.npy")
c = T.mean(1)
fig = plt.figure(figsize=(18, 10))
sel = (c[:, 1] < -30) & (c[:, 0] > -30) & (c[:, 0] < 135)
for i, (box, el, az, ttl) in enumerate([
        ((-25, 5, -55, -25, 222, 250), 15, -50, "corner -X/-Z (iso)"),
        ((-25, 5, -55, -25, 222, 250), 0, -90, "corner -X/-Z (look along +Z)"),
        ((-30, 135, -55, -30, 210, 360), 89, -90, "bottom y<-30 top-down"),
        ((-30, 135, -55, -30, 210, 360), 0, 0, "bottom y<-30 side")]):
    x0, x1, y0, y1, z0, z1 = box
    m = (c[:, 0] > x0) & (c[:, 0] < x1) & (c[:, 1] > y0) & (c[:, 1] < y1) & (c[:, 2] > z0) & (c[:, 2] < z1)
    P = T[m][:, :, [0, 2, 1]]
    ax = fig.add_subplot(2, 2, i + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(P, facecolor=plt.cm.tab20(fid[m] % 20 / 20), edgecolor="k", linewidth=0.05, alpha=0.8))
    r = max(x1 - x0, y1 - y0, z1 - z0) / 2; cc = ((x0 + x1) / 2, (z0 + z1) / 2, (y0 + y1) / 2)
    ax.set_xlim(cc[0]-r, cc[0]+r); ax.set_ylim(cc[1]-r, cc[1]+r); ax.set_zlim(cc[2]-r, cc[2]+r)
    ax.view_init(el, az); ax.set_title(ttl); ax.set_xlabel("X"); ax.set_ylabel("Z"); ax.set_zlabel("Y")
plt.tight_layout(); plt.savefig("bottom.png", dpi=65)
