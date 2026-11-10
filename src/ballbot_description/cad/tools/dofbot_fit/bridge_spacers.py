import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Bridge on v1's 'm1.5 rubber spacer' x4 (0.5 mm) instead of the retired 5 mm standoffs. ~20 calls.

Run after setting T_bridge_L.npy (check_top.py). The header-pin slot is in the v2 cover (out/orin_cover.step).
Moves the bridge's 'mc bridge target' connector to the new pose, inserts the spacers and fastens each to the
housing base under a bridge hole. The bridge standoff instances are deleted in the UI. Resumable via placed.json.
"""
import json, sys, numpy as np
from urllib.parse import quote
from bb import api, D
from place import W, HOUSING, X, T, asm, state, save
import mates as m
sys.path.insert(0, m.__file__.rsplit("/", 3)[0] + "/parts")
from orin_housing import BRIDGE_HOLES, Z_TOP

SPACER = {"elementId": "453045936effd264e06b07fe", "partId": "JHD"}  # v1, 2.5 dia x 0.5, origin bottom centre
sp = state.setdefault("spacers", {})
names = {i["id"]: i["name"] for i in json.load(open("housing_named.json"))["rootAssembly"]["instances"]}
m.rep["housing"] = [i for i in m.rep["housing"] if not names.get(i, "").startswith("standoff_m2x5_bridge")]
Wi = np.linalg.inv(m.mm(m.H[(m.BASE,)]))

if not sp.get("bridge_moved"):
    fid = m.done["mc bridge target"]
    f = {**m.connector("mc bridge target", (m.BASE,), Wi @ X["bridge"]), "featureId": fid}
    r = api.post(asm(HOUSING) + "/features/featureid/" + quote(fid, safe=""), json={"feature": f}).json()
    st = (r.get("featureState") or {}).get("featureStatus"); print("mc bridge target", st)
    if st != "OK": sys.exit("bridge connector update failed")
    sp["bridge_moved"] = True; save()
if "ids" not in sp:
    ids0 = {i["id"] for i in api.get(asm(HOUSING)).json()["rootAssembly"]["instances"]}
    for _ in BRIDGE_HOLES: api.post(asm(HOUSING) + "/instances", json={"documentId": D, **SPACER})
    sp["ids"] = [i["id"] for i in api.get(asm(HOUSING)).json()["rootAssembly"]["instances"] if i["id"] not in ids0]
    assert len(sp["ids"]) == len(BRIDGE_HOLES), sp["ids"]; save()
for k, (iid, (x, y)) in enumerate(zip(sp["ids"], BRIDGE_HOLES)):
    n = f"bridge spacer {k + 1}"
    if f"fasten {n}" in m.done: continue
    a = m.mc(HOUSING, f"mc {n}", (iid,))
    b = m.mc(HOUSING, f"mc {n} target", (m.BASE,), Wi @ X["housing"] @ T([x, y, Z_TOP]))
    m.done[f"fasten {n}"] = m.add(HOUSING, m.mate(f"fasten {n}", m.fq(a), m.fq(b))); save()
print(json.dumps(sp, indent=1))
