import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import numpy as np
from build123d import import_step
from reg import *
X = "/home/atticus/Downloads/cad/dofbot-pro/extracted/"
frame = pts(import_step(X + "orin_enclosure_assembly-frame.step"), 15000)
hous = import_step("housing.step")
shell = next(c for c in hous.children if c.label == "orin_enclosure")
sh = pts(shell, 15000)
s, R, t, med, p90 = register(frame, sh, frame.mean(0), sh.mean(0))
print("shell fit: inlier", round(s, 3), "median", round(med, 4), "p90", round(p90, 4))
M = H(R, t); print(np.round(M, 4)); np.save("M_fused_to_housing.npy", M)
