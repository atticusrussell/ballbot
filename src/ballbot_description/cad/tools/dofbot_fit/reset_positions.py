import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Drop electronics_group and re-set every placement in dofbot_pcb_housing. ~8 calls."""
import json, numpy as np
from bb import api
from place import asm, HOUSING, X, T, set_xf, state
from urllib.parse import quote
rep = state["repair"]
for f in api.get(asm(HOUSING) + "/features").json()["features"]:
    if f.get("name") == "electronics_group":
        api.delete(asm(HOUSING) + "/features/featureid/" + quote(f["featureId"], safe=""))
H = {tuple(o["path"]): np.array(o["transform"]).reshape(4, 4) for o in api.get(asm(HOUSING)).json()["rootAssembly"]["occurrences"]}
def m(M): N = M.copy(); N[:3, 3] /= 1000; return N
def mm(M): N = M.copy(); N[:3, 3] *= 1000; return N
# sub-assemblies: relative delta from where they are now (rigid moves keep internals)
for inst, tgt in ((rep["orin"], X["orin"]), (rep["pcb"], X["pcb"])):
    delta = tgt @ np.linalg.inv(mm(H[(inst,)]))
    if np.abs(delta - np.eye(4)).max() > 1e-6: set_xf(HOUSING, [[inst]], delta, relative=True)
# flat parts: absolute is safe
set_xf(HOUSING, [[i] for i in rep["housing"]], X["housing"])
set_xf(HOUSING, [[i] for i in state["a1"]], X["a1"])
set_xf(HOUSING, [[i] for i in state["bridge"]], X["bridge"])
names = {i["id"]: i["name"] for i in api.get(asm(HOUSING)).json()["rootAssembly"]["instances"]}
s15 = [[i] for i in rep["stands"] if not names[i].startswith("stand-8")]
s8 = [[i] for i in rep["stands"] if names[i].startswith("stand-8")]
set_xf(HOUSING, s15, X["a1"]); set_xf(HOUSING, s8, T([0, 0, 1.6]) @ X["a1"])
print("reset done")
