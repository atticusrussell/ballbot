import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Snap the stray D23 <2> leaf (in its D23 <2> sub-assembly of 600-13768) back onto that sub-assembly, in every context.

orin_assm held an override putting the leaf ~120 mm off; siblings sit at identity in their D23 sub. ~5-9 calls.
"""
import numpy as np
from urllib.parse import quote
from bb import api
from place import asm, set_xf
import mates as m
SUB, LEAF = "Mwwb0bKpgg6JF10kK", "M6+csY7XsdLixVCjA"
CTX = {"orin": "db2e80cc3658263454d68db6", "housing": "76281463014c048875567be7",
       "chassis": "69b4604a7be4b0e3dbe3d002", "robot": "83ac8f3e1b8a64890594cf88"}

def offs(e):  # [(leaf path, delta world-mm moving leaf onto its sub)]
    r = api.get(asm(e) + "?includeNonSolids=true").json()["rootAssembly"]["occurrences"]
    occ = {tuple(o["path"]): m.mm(np.array(o["transform"]).reshape(4, 4)) for o in r}
    out = []
    for p, M in occ.items():
        if p[-2:] == (SUB, LEAF):
            d = occ[p[:-1]] @ np.linalg.inv(M)
            out.append((p, d, np.abs(d - np.eye(4)).max()))
    return out

def group(e, sup):  # 'orin rigid' ties 600-13768 to the fixed Orin module -> leaf counts as fixed; toggle around the move
    f = [f for f in api.get(asm(e) + "/features").json()["features"] if f["name"] == "orin rigid"][0]
    f["suppressed"] = sup
    r = api.post(asm(e) + "/features/featureid/" + quote(f["featureId"], safe=""), json={"feature": f}).json()
    print("orin rigid suppressed" if sup else "orin rigid restored", (r.get("featureState") or {}).get("featureStatus"))

for k, e in CTX.items():
    o = offs(e)
    print(k, [round(x[2], 4) for x in o])
    bad = [x for x in o if x[2] > 1e-3]
    if not bad: continue
    if k == "orin": group(e, True)
    try:
        for p, d, _ in bad: set_xf(e, [list(p)], d, relative=True)
    finally:
        if k == "orin": group(e, False)
    print(k, "after", [round(x[2], 4) for x in offs(e)])
