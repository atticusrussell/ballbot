import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Lidar as v1's 'RPlidar A1 Base' + 'RPlidar A1 Head' sub-assemblies on 20+5 mm standoffs (no dev-kit board/stands).

Run check_lidar.py first (writes T_a1_L.npy). Pushes out/lidar_standoffs.step, inserts Base/Head at the A1 pose,
the standoffs in frame L, and fastened-mates them (Base -> housing base, Head -> Base, standoffs -> housing base).
The 30 loose A1 kit instances (and their mates) are deleted in the UI. Resumable via placed.json ('lidar'). ~40 calls.
"""
import json, os, subprocess, sys, numpy as np
from bb import api, D
from place import W, HOUSING, CAD, X, asm, insert, set_xf, state, save
import mates as m

A1_BASE, A1_HEAD = "aafba4890f5b8a8cf69104ff", "fb011d5b2b7e89ea520ff3e9"
BASE_LEAF, HEAD_LEAF = "MZBeORmRkwRTMk157", "Mv/bUwq+T/dAD6EJO"  # base.ipt, laser-mount.ipt (both Z up)
lid = state.setdefault("lidar", {})
def leaf(name, iid): return m.mm(np.array(next(o["transform"] for o in json.load(open(f"a1_{name}_asm.json"))[
    "rootAssembly"]["occurrences"] if o["path"] == [iid])).reshape(4, 4))
S_base, S_head = leaf("base", BASE_LEAF), leaf("head", HEAD_LEAF)  # sub-assembly coords = onepart studio coords

if "ps" not in lid:
    step = os.path.join(CAD, "out/lidar_standoffs.step")
    subprocess.run([sys.executable, os.path.join(CAD, "tools/onshape_step_sync.py"), "push", "--new",
                    f"https://cad.onshape.com/documents/{D}/w/{W}/e/{HOUSING}", step], check=True)
    rec = next(v for k, v in json.load(open(os.path.join(CAD, "out/.onshape_imports.json"))).items()
               if k.endswith("lidar_standoffs.step"))
    lid["ps"] = next(e["id"] for e in rec["resultElements"] if e["type"] == "PARTSTUDIO"); save()
for key, eid in (("base", A1_BASE), ("head", A1_HEAD)):
    if key not in lid:
        lid[key] = insert(HOUSING, {"elementId": eid, "isAssembly": True})[0]["id"]; save()
        set_xf(HOUSING, [[lid[key]]], X["a1"], relative=True)  # fresh insert is identity
if "standoffs" not in lid:
    lid["standoffs"] = [i["id"] for i in insert(HOUSING, {"elementId": lid["ps"], "isWholePartStudio": True})]
    set_xf(HOUSING, [[i] for i in lid["standoffs"]], X["housing"]); save()  # top-level parts: absolute is safe
print(json.dumps(lid, indent=1))

Wb = m.mm(m.H[(m.BASE,)])
plan = [("lidar base", (lid["base"], BASE_LEAF), (m.BASE,), np.linalg.inv(Wb) @ X["a1"] @ S_base),
        ("lidar head", (lid["head"], HEAD_LEAF), (lid["base"], BASE_LEAF), np.linalg.inv(S_base) @ S_head)]
plan += [(f"lidar standoff {i}", (s,), (m.BASE,), None) for i, s in enumerate(lid["standoffs"])]
for n, c, par, off in plan:
    if f"fasten {n}" in m.done: continue
    a = m.mc(HOUSING, f"mc {n}", c)
    b = m.mc(HOUSING, f"mc {n} target", par, off) if off is not None else m.mc(HOUSING, f"mc origin {par[-1]}", par)
    m.done[f"fasten {n}"] = m.add(HOUSING, m.mate(f"fasten {n}", m.fq(a), m.fq(b))); save()
