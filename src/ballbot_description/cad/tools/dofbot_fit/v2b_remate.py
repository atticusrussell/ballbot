import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""v2b: PCB + Orin turned 180 about the board centre. ~25 calls, resumable ('v2b').

Before: orin_housing.py with BOARD_ROT, board_bosses.py sketch updates, new orin_cover import; in the UI delete the old
cover / PCB / orin_assm (fresh inserts also drop their detached leaves) and insert orin_cover (1), YB-EVV01 PCB,
orin_assm. Moves the standoff targets, re-posts the floor-insert and Orin-screw mates, mates cover, PCB, Orin.
"""
import json, sys, numpy as np
from urllib.parse import quote
sys.argv.append("--plan-only")
from bb import api
from place import HOUSING, X, T, asm, state, save, ROT_H
import mates as m
import hw_mates as h
from orin_housing import BOARD_HOLES, ORIN_HOLES, PCB_Z, PCB_T
v = state.setdefault("v2b", {}); rep = state["repair"]; done = m.done; v2 = state["v2"]
RY = h.RY90
mm = m.mm
if "ids" not in v:
    inst = api.get(asm(HOUSING) + "?includeNonSolids=true").json()["rootAssembly"]["instances"]
    one = lambda p: [i["id"] for i in inst if i["name"].startswith(p)]
    v["old"] = {"pcb": rep["pcb"], "orin": rep["orin"], "cover": v2["ids"]["cover"]}
    v["ids"] = {"pcb": one("YB-EVV01 PCB")[0], "orin": one("orin_assm")[0], "cover": one("orin_housing_cover")[0]}
    assert all(len(one(p)) == 1 for p in ("YB-EVV01 PCB", "orin_assm", "orin_housing_cover")); save()
ids = v["ids"]
def post(fid, f):
    f["featureId"] = fid; r = api.post(asm(HOUSING) + "/features/featureid/" + quote(fid, safe=""), json={"feature": f}).json()
    st = (r.get("featureState") or {}).get("featureStatus"); print(f"  {f['name']:<32} {st}")
    if st != "OK": sys.exit(f"{f['name']} failed")
z_orin = PCB_Z + PCB_T + 10
targets = {f"board standoff {k + 1}": (hh, PCB_Z) for k, hh in enumerate(BOARD_HOLES)}
targets.update({f"orin standoff {k + 1}": (hh, z_orin) for k, hh in enumerate(ORIN_HOLES)})
XL = np.linalg.inv(mm(m.H[(m.BASE,)])) @ X["housing"]
def conn(name, M):  # = board_standoffs.conn (target on the base origin, T @ Ry(90))
    t = M[:3, 3]; q = m.q
    return {"btType": "BTMMateConnector-66", "featureType": "mateConnector", "name": name, "parameters": [
        {"btType": "BTMParameterEnum-145", "parameterId": "originType", "enumName": "Origin type", "value": "ON_ENTITY"},
        {"btType": "BTMParameterQueryWithOccurrenceList-67", "parameterId": "originQuery", "queries": [m.origin((m.BASE,))]},
        {"btType": "BTMParameterBoolean-144", "parameterId": "transform", "value": True},
        q("translationX", f"{t[0]:.6f} mm"), q("translationY", f"{t[1]:.6f} mm"), q("translationZ", f"{t[2]:.6f} mm"),
        {"btType": "BTMParameterEnum-145", "parameterId": "rotationType", "enumName": "Rotation axis", "value": "ABOUT_Y"},
        q("rotation", "90 deg")]}
for n, ((x, y), z) in targets.items():  # 1. standoff targets to the turned holes
    if n in v.setdefault("moved", []): continue
    post(done[f"mc {n} target"], conn(f"mc {n} target", XL @ T([x, y, z], RY)))
    v2["standoffs"][n][1] = [x, y]; v["moved"].append(n); save()
sys.argv.remove("--plan-only")
for name, child, parent in list(h.PLAN):  # 2. floor inserts + Orin screws follow the holes
    if not name.startswith(("fasten floor insert", "fasten orin screw")) or name in v.setdefault("reposted", []): continue
    # updating a mate with implicit connectors errors (ids re-mapped); delete + re-add instead
    api.delete(asm(HOUSING) + "/features/featureid/" + quote(done.pop(name), safe="")); save()
    h.fasten(HOUSING, name, child, parent); v["reposted"].append(name); save()
occ = {tuple(o["path"]): mm(np.array(o["transform"]).reshape(4, 4)) for o in json.load(open("housing_v2_now.json"))["rootAssembly"]["occurrences"]}
S = lambda n: X["housing"] @ T([*targets[n][0], targets[n][1]], RY)  # standoff pose (housing)
board, mtt = "MsqHB/2KD9gNBwaQx", "MttAisj6ynTB7npZt"
Pb = ROT_H @ occ[(v["old"]["pcb"], board)]; Po = ROT_H @ occ[(v["old"]["orin"], mtt)]  # turned verified poses
Wb = X["housing"] @ T([*targets["board standoff 1"][0], PCB_Z])  # hole 1 bottom = standoff top
Wo = X["housing"] @ T([*targets["orin standoff 1"][0], z_orin + 10])  # Orin carrier bottom on its standoff
s1, o1 = v2["standoffs"]["board standoff 1"][0], v2["standoffs"]["orin standoff 1"][0]
h.fasten(HOUSING, "fasten cover v2b", ((ids["cover"],), np.eye(4)), ((m.BASE,), XL))
h.fasten(HOUSING, "fasten pcb v2b", ((ids["pcb"], board), np.linalg.inv(Pb) @ Wb), ((s1,), np.linalg.inv(S("board standoff 1")) @ Wb))
h.fasten(HOUSING, "fasten orin v2b", ((ids["orin"], mtt), np.linalg.inv(Po) @ Wo), ((o1,), np.linalg.inv(S("orin standoff 1")) @ Wo))
rep["pcb"], rep["orin"], v2["ids"]["cover"] = ids["pcb"], ids["orin"], ids["cover"]; save()
