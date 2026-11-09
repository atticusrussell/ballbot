import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Re-insert the YB-EVV01 PCB sub-assembly with a relative transform (absolute/leaf moves scramble it). ~9 calls."""
import json, numpy as np
from bb import api, D
from place import asm, HOUSING, X, insert, set_xf, state, save
from repair import group
from urllib.parse import quote
PCB_EID, OLD = "fc16901070e8ac7eef2142d4", "MVWER4al7DjyElyq2"
rep = state["repair"]
feats = api.get(asm(HOUSING) + "/features").json()["features"]
for f in feats:
    if f.get("name") == "electronics_group":
        api.delete(asm(HOUSING) + "/features/featureid/" + quote(f["featureId"], safe=""))
api.delete(asm(HOUSING) + f"/instance/nodeid/{rep.get('pcb', OLD)}")
rep["pcb"] = insert(HOUSING, {"elementId": PCB_EID, "isAssembly": True})[0]["id"]; save()
set_xf(HOUSING, [[rep["pcb"]]], X["pcb"], relative=True)
r0 = json.load(open("housing_asm.json"))
P0 = {tuple(o["path"]): np.array(o["transform"]).reshape(4, 4) for o in r0["rootAssembly"]["occurrences"]}
Dp = np.load("D_pcb_snapped.npy"); Dp[:3, 3] /= 1000
H = {tuple(o["path"]): np.array(o["transform"]).reshape(4, 4) for o in api.get(asm(HOUSING)).json()["rootAssembly"]["occurrences"]}
off = [k for k, v in P0.items() if k[0] == OLD and len(k) > 1 and np.abs(H[(rep["pcb"],) + k[1:]] - Dp @ v).max() > 1e-6]
print("PCB leaves off:", len(off), "of", sum(1 for k in P0 if k[0] == OLD and len(k) > 1))
if not off:
    paths = [[rep["pcb"]], [rep["orin"]]] + [[i] for i in rep["housing"] + state["a1"] + rep["stands"] + state["bridge"]]
    group(HOUSING, paths, "electronics_group"); print("group re-added")
print(json.dumps({k: rep[k] for k in ("pcb",)}))
