import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Replace Fix/Group locking with fastened mates in dofbot_pcb_housing and still_chassis_assm. ~1 call per mate.

One fixed ground part per assembly (orin_housing_base; v1's plate assm in the chassis). Parts from one Part Studio
share a part origin, so they mate origin-to-origin with no offset. Everything else mates to the ground through a
mate connector moved by the current relative pose (translation + rotation about Z only, checked).
Resumable via placed.json ('mates' dict). --dry prints the plan only.
"""
import json, sys, numpy as np
from urllib.parse import quote
from bb import api, D
from place import W, HOUSING, CHASSIS, X, asm, state, save
from scipy.spatial.transform import Rotation as R

rep = state["repair"]; done = state.setdefault("mates", {})
H = {tuple(k.split("|")): np.array(v) for k, v in json.load(open("verify_housing.json"))["H"].items()}
named = json.load(open("housing_named.json"))["rootAssembly"]["instances"]
by_name = {i["name"]: i["id"] for i in named}
BASE = by_name["orin_housing_base <1>"]; A1_ANCHOR = by_name["base.ipt <1>"]
ORIN_LEAF, PCB_LEAF = "MttAisj6ynTB7npZt", None  # first part of orin_assm; PCB leaf picked below

def mm(M): N = M.copy(); N[:3, 3] *= 1000; return N
def origin(path): return {"btType": "BTMInferenceQueryWithOccurrence-1083", "inferenceType": "PART_ORIGIN",
                          "path": list(path), "deterministicIds": [""]}
def q(name, v): return {"btType": "BTMParameterQuantity-147", "parameterId": name, "expression": v}
def connector(name, path, M):  # mate connector at a part origin, moved by M (mm, Z-rotation only)
    flip = M[2, 2] < 0  # flipPrimary = 180 deg about X, applied before the move
    if flip: F = np.diag([1, -1, -1]); M = M.copy(); M[:3, :3] = F @ M[:3, :3]; M[:3, 3] = F @ M[:3, 3]
    rv = R.from_matrix(M[:3, :3]).as_rotvec(degrees=True)
    assert abs(rv[0]) < 1e-6 and abs(rv[1]) < 1e-6, (name, rv)
    t = M[:3, 3]
    move = [] if np.allclose(M, np.eye(4)) else [
        {"btType": "BTMParameterBoolean-144", "parameterId": "transform", "value": True},
        q("translationX", f"{t[0]:.6f} mm"), q("translationY", f"{t[1]:.6f} mm"), q("translationZ", f"{t[2]:.6f} mm"),
        {"btType": "BTMParameterEnum-145", "parameterId": "rotationType", "enumName": "Rotation axis",
         "value": "ABOUT_Z"},
        q("rotation", f"{rv[2]:.6f} deg")]
    return {"btType": "BTMMateConnector-66", "featureType": "mateConnector", "name": name, "parameters": [
        {"btType": "BTMParameterEnum-145", "parameterId": "originType", "enumName": "Origin type", "value": "ON_ENTITY"},
        {"btType": "BTMParameterQueryWithOccurrenceList-67", "parameterId": "originQuery", "queries": [origin(path)]},
        *([{"btType": "BTMParameterBoolean-144", "parameterId": "flipPrimary", "value": True}] if flip else []),
        *move]}
def mate(name, qa, qb):
    return {"btType": "BTMMate-64", "featureType": "mate", "name": name, "parameters": [
        {"btType": "BTMParameterEnum-145", "parameterId": "mateType", "enumName": "Mate type", "value": "FASTENED"},
        {"btType": "BTMParameterQueryWithOccurrenceList-67", "parameterId": "mateConnectorsQuery", "queries": [qa, qb]}]}
def fq(fid): return {"btType": "BTMFeatureQueryWithOccurrence-157", "featureId": fid, "path": [], "queryData": ""}
def add(e, f):
    r = api.post(asm(e) + "/features", json={"feature": f}).json()
    st = (r.get("featureState") or {}).get("featureStatus")
    print(f"  {f['name']:<40} {r['feature']['featureId']}  {st}")
    if st != "OK":
        api.delete(asm(e) + "/features/featureid/" + quote(r["feature"]["featureId"], safe=""))
        sys.exit(f"{f['name']} did not resolve ({r.get('featureState')}); deleted it, stopping")
    return r["feature"]["featureId"]

def plan():
    Wb = mm(H[(BASE,)])
    pcb = rep["pcb"]
    pcb_leaf = (pcb, "MsqHB/2KD9gNBwaQx")  # Board.step
    p = []  # (name, child path, parent path, offset in parent frame or None)
    for i in rep["housing"]:
        if i != BASE: p.append((f"housing {i}", (i,), (BASE,), None))
    for i in st_a1():
        if i != A1_ANCHOR: p.append((f"a1 kit {i}", (i,), (A1_ANCHOR,), None))
    for n, path in (("a1 kit anchor", (A1_ANCHOR,)), ("bridge", (state["bridge_comp"],)),
                    ("orin", (rep["orin"], ORIN_LEAF)), ("pcb", pcb_leaf)):
        p.append((n, path, (BASE,), np.linalg.inv(Wb) @ mm(H[path])))
    for n, c, _, off in p:  # no-offset mates need coincident origins
        if off is None: assert np.abs(mm(H[c]) - mm(H[_])).max() < 1e-6, n
    return p
def st_a1(): return state["a1"] + rep["stands"]

def mc(e, key, path, M=np.eye(4)):  # connector feature, cached in placed.json by key
    if key not in done: done[key] = add(e, connector(key, path, M)); save()
    return done[key]
def chassis():  # housing copy's base part -> a plate assm part (v1 ground), offset connector on the plate side
    a = api.get(asm(CHASSIS) + "?includeNonSolids=true").json(); json.dump(a, open("chassis_named.json", "w"))
    C = {tuple(o["path"]): mm(np.array(o["transform"]).reshape(4, 4)) for o in a["rootAssembly"]["occurrences"]}
    plate = next(i for i in a["rootAssembly"]["instances"] if i["name"].startswith("plate assm"))
    sub = next(s for s in a["subAssemblies"] if s["elementId"] == plate["elementId"])
    leaf = next((plate["id"], i["id"]) for i in sub["instances"] if i["type"] == "Part" and "plate" in i["name"].lower())
    c = (rep["chassis"], BASE)
    off = np.linalg.inv(C[leaf]) @ C[c]
    print("plate leaf", leaf, "offset", np.round(off, 3))
    if "fasten chassis" in done: return
    ca = mc(CHASSIS, "mc dofbot_pcb_housing", c); cb = mc(CHASSIS, "mc dofbot_pcb_housing target", leaf, off)
    done["fasten chassis"] = add(CHASSIS, mate("fasten dofbot_pcb_housing", fq(ca), fq(cb))); save()

def main():
    if "--chassis" in sys.argv: return chassis()
    p = plan()
    print(f"{len(p)} mates in dofbot_pcb_housing, {sum(o is not None for *_, o in p)} with offset connectors")
    if "--dry" in sys.argv:
        for n, c, par, off in p: print(n, c, "->", par, None if off is None else np.round(off[:3, 3], 3))
        return
    p.sort(key=lambda x: x[3] is None)  # offset mates first (riskier)
    for n, c, par, off in p:
        if f"fasten {n}" in done: continue
        a = mc(HOUSING, f"mc {n}", c)
        b = mc(HOUSING, f"mc {n} target", par, off) if off is not None else mc(HOUSING, f"mc origin {par[-1]}", par)
        done[f"fasten {n}"] = add(HOUSING, mate(f"fasten {n}", fq(a), fq(b))); save()
        if "--one" in sys.argv: return


if __name__ == "__main__":
    main()
