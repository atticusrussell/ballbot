import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Verify Onshape positions: every leaf of Orin/PCB/housing/lidar base/v2 standoffs/bridge vs intended, and the chassis copy. 3 calls."""
import json, sys, numpy as np
from bb import api, D
from place import W, HOUSING, CHASSIS, ORIN, X, T, CAD, asm
sys.path.insert(0, CAD + "/parts")
st = json.load(open("placed.json")); rep = st["repair"]
occ = lambda r: {tuple(o["path"]): np.array(o["transform"]).reshape(4, 4) for o in r["rootAssembly"]["occurrences"]}
mm = lambda M: (lambda N: (N.__setitem__((slice(0, 3), 3), N[:3, 3] * 1000), N)[1])(M.copy())
H = occ(api.get(asm(HOUSING) + "?includeNonSolids=true").json()); O = occ(api.get(asm(ORIN)).json()); C = occ(api.get(asm(CHASSIS) + "?includeNonSolids=true").json())
json.dump({"H": {"|".join(k): v.tolist() for k, v in H.items()}}, open("verify_housing.json", "w"))
def err(a, b): return np.abs(mm(a) - mm(b)).max()
# Orin: each leaf = D_orin @ its pose inside orin_assm
e = max(err(H[(rep["orin"],) + k], (lambda M: (M.__setitem__((slice(0, 3), 3), M[:3, 3] / 1000), M)[1])(X["orin"].copy()) @ v) for k, v in O.items())
print(f"Orin: {len(O)} leaves, max error {e:.4f} mm")
from orin_housing import RAISE, BRIDGE_HOLES, Z_TOP
P0 = occ(json.load(open("housing_asm.json"))); Dp = T([0, 0, RAISE]) @ np.load("D_pcb_snapped.npy"); Dp[:3, 3] /= 1000
pi = rep.get("pcb", "MVWER4al7DjyElyq2")
e = max(err(H[(pi,) + k[1:]], Dp @ v) for k, v in P0.items() if k[0] == "MVWER4al7DjyElyq2" and len(k) > 1)
print(f"PCB: max error {e:.4f} mm")
lid = st["lidar"]
for sub in ("base",):  # lidar sub-assembly leaves (head: robot_assm, robot_head.py) = A1 pose @ leaf pose inside RPlidar A1 Base/Head
    L = occ(json.load(open(f"a1_{sub}_asm.json"))); Xa = X["a1"].copy(); Xa[:3, 3] /= 1000
    print(f"lidar {sub}: {len(L)} leaves, max error {max(err(H[(lid[sub],) + k], Xa @ v) for k, v in L.items()):.4f} mm")
for name, ids, M in (("housing base + cover", [i for i in rep["housing"] if (i,) in H] + [st["v2"]["ids"]["cover"]], X["housing"]),
                     ("bridge composite", [st["bridge_comp"]], X["bridge"])):
    Mm = M.copy(); Mm[:3, 3] /= 1000
    print(f"{name}: {len(ids)} parts, max error {max(err(H[(i,)], Mm) for i in ids):.4f} mm")
e = max(err(H[(i,)], (lambda M: (M.__setitem__((slice(0, 3), 3), M[:3, 3] / 1000), M)[1])(X["housing"] @ T([x, y, Z_TOP])))
        for i, (x, y) in zip(st["spacers"]["ids"], BRIDGE_HOLES))
print(f"bridge spacers: {len(BRIDGE_HOLES)} parts, max error {e:.4f} mm")
RY = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)  # doc standoffs: part +X (stud) down
e = max(err(H[(i,)], (lambda M: (M.__setitem__((slice(0, 3), 3), M[:3, 3] / 1000), M)[1])(X["housing"] @ T([h[0], h[1], z], RY)))
        for i, h, z in st["v2"]["standoffs"].values())
print(f"doc standoffs: {len(st['v2']['standoffs'])} parts, max error {e:.4f} mm")
Xc = X["chassis"].copy(); Xc[:3, 3] /= 1000; ci = rep["chassis"]
e = max(err(C[(ci,) + k], Xc @ v) for k, v in H.items())
print(f"chassis copy: {sum(1 for k in C if k[0] == ci)} leaves vs {len(H)}, max error {e:.4f} mm")
