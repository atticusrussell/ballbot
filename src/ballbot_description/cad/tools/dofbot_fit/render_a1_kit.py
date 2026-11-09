import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Render the RP-lidar-A1 onepart export with the kit devboard / stands highlighted -> a1_kit.png."""
import sys, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from build123d import import_step
from meshlib import tris
COL = {"devboard": "red", "connector-7pin": "orange", "stand-8": "blue", "stand_15": "deepskyblue"}
a1 = import_step("a1.step")
fig = plt.figure(figsize=(14, 7))
for n, (elev, azim) in enumerate(((20, -60), (-25, -60))):
    ax = fig.add_subplot(1, 2, n + 1, projection="3d")
    allp = []
    for c in a1.children:
        col = next((v for k, v in COL.items() if c.label.startswith(k)), "lightgrey")
        t, _ = tris(c.wrapped, 0.8)
        if len(t) == 0: continue
        ax.add_collection3d(Poly3DCollection(t, facecolor=col, edgecolor="none", alpha=1.0 if col != "lightgrey" else 0.35))
        allp.append(t.reshape(-1, 3))
    P = np.vstack(allp); lo, hi = P.min(0), P.max(0); c0 = (lo + hi) / 2; r = (hi - lo).max() / 2
    ax.set_xlim(c0[0] - r, c0[0] + r); ax.set_ylim(c0[1] - r, c0[1] + r); ax.set_zlim(c0[2] - r, c0[2] + r)
    ax.view_init(elev, azim); ax.set_axis_off(); ax.set_title("from above" if n == 0 else "from below")
fig.legend(handles=[plt.Rectangle((0, 0), 1, 1, color=v) for v in COL.values()] + [plt.Rectangle((0, 0), 1, 1, color="lightgrey")],
           labels=["devboard.ipt (left out)", "connector-7pin (left out)", "stand-8x2 (kit)", "stand_15x2 (kit)", "A1 lidar"], loc="lower center", ncol=5)
out = sys.argv[1] if len(sys.argv) > 1 else "a1_kit.png"
plt.savefig(out, dpi=110, bbox_inches="tight"); print(out)
