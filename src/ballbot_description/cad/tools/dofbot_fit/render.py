import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import sys, numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape
from meshlib import tris
s = TopoDS_Shape(); BRepTools.Read_s(s, "region.brep", BRep_Builder())
T, fid = tris(s, 0.8); np.save("region_tris.npy", T); np.save("region_fid.npy", fid)
print(T.shape)
fig = plt.figure(figsize=(16, 12))
views = [(20, -60, "iso"), (90, -90, "top (+Y up => looking -Y)"), (0, -90, "front (looking +Z?)"), (0, 0, "side")]
# frame coords are Y-up: map plot (x, z, y) so Y is vertical
P = T[:, :, [0, 2, 1]]
for i, (el, az, name) in enumerate(views):
    ax = fig.add_subplot(2, 2, i + 1, projection="3d")
    ax.add_collection3d(Poly3DCollection(P, facecolor=plt.cm.tab20(fid % 20 / 20), edgecolor="none", alpha=0.6))
    mn, mx = P.reshape(-1, 3).min(0), P.reshape(-1, 3).max(0); c = (mn + mx) / 2; r = (mx - mn).max() / 2
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.view_init(el, az); ax.set_title(name); ax.set_xlabel("X"); ax.set_ylabel("Z"); ax.set_zlabel("Y")
plt.tight_layout(); plt.savefig("region.png", dpi=70)
