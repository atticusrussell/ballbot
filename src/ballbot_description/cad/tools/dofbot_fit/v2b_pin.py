import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Pin the independent top-level occurrences of orin_assm (7) and the PCB's SOT-223 with fastened mates to the base.

Only the mated leaf follows a mate; these occurrences move on their own in every context (the old 'detached
leaves'). Each gets an implicit-connector fasten at its verified pose turned 180 (v2b). 8 calls.
"""
import json, sys, numpy as np
sys.argv.append("--plan-only")
from place import HOUSING, X, state, save, ROT_H
from scipy.spatial.transform import Rotation as R
import mates as m
import hw_mates as h
sys.argv.remove("--plan-only")
v = state["v2b"]; mm = m.mm
old = {tuple(o["path"]): mm(np.array(o["transform"]).reshape(4, 4)) for o in json.load(open("housing_v2_now.json"))["rootAssembly"]["occurrences"]}
orin_top = [i["id"] for i in json.load(open("orin_asm_now.json"))["rootAssembly"]["instances"] if i["id"] != "MttAisj6ynTB7npZt"]
Wb = mm(m.H[(m.BASE,)])
def one_axis(M):
    for fl in (np.eye(3), np.diag([1, -1, -1])):
        rv = R.from_matrix(fl @ M[:3, :3]).as_rotvec()
        if sum(abs(x) > 1e-6 for x in rv) <= 1: return True
    return False
parents = {k[:-1] for k in old}  # leaves = paths nothing else extends (parts, not sub-assemblies)
jobs = []
for nm, tops in (("orin", orin_top),):  # SOT-223: tilted part frame, see v2b_sot.py
    for t in tops:  # first axis-aligned leaf under this top-level occurrence
        k = next(k for k in sorted(old, key=len) if k[0] == v["old"][nm] and len(k) > 1 and k[1] == t and k not in parents and one_axis(old[k]))
        jobs.append((f"pin {nm} {t}", (v["ids"][nm],) + k[1:], ROT_H @ old[k]))
for n, path, P in jobs:
    h.fasten(HOUSING, n, (path, np.eye(4)), ((m.BASE,), np.linalg.inv(Wb) @ P))
