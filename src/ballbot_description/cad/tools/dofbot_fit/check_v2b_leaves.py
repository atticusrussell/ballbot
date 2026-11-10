import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""v2b: every Orin / PCB leaf vs its verified v2 pose turned 180 (1 call)."""
import json, numpy as np
from bb import api
from place import HOUSING, asm, state, ROT_H
import mates as m
v = state["v2b"]
old = {tuple(o["path"]): m.mm(np.array(o["transform"]).reshape(4, 4)) for o in json.load(open("housing_v2_now.json"))["rootAssembly"]["occurrences"]}
a = api.get(asm(HOUSING) + "?includeNonSolids=true").json(); json.dump(a, open("housing_v2b_now.json", "w"))
H = {tuple(o["path"]): m.mm(np.array(o["transform"]).reshape(4, 4)) for o in a["rootAssembly"]["occurrences"]}
for nm in ("orin", "pcb"):
    e = sorted(((np.abs(H[(v["ids"][nm],) + k[1:]] - ROT_H @ M).max(), k[1:]) for k, M in old.items()
                if k[0] == v["old"][nm] and len(k) > 1), reverse=True)
    print(f"{nm}: {len(e)} leaves, bad {sum(x > 1e-3 for x, _ in e)}, max {e[0][0]:.4f} mm", e[0][1] if e[0][0] > 1e-3 else "")
