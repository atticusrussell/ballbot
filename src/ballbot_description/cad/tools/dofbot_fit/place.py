import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Place everything in dofbot_pcb_housing (and the housing in still_chassis_assm). ~30 API calls.

Steps are idempotent via placed.json; rerun skips finished steps. --dry prints transforms only.
"""
import json, os, subprocess, sys, numpy as np
from bb import api, D
from check import t_shell
W = "9dc077545bceb395c1a5f5aa"
HOUSING, CHASSIS = "76281463014c048875567be7", "69b4604a7be4b0e3dbe3d002"
ORIN, A1, BRIDGE = "db2e80cc3658263454d68db6", "da19edeb6e5210e891cf1287", "5cc94e7f59506072a89e0c08"
PCB_INST = "MVWER4al7DjyElyq2"
A1_DROP = ("devboard", "connector-7pin")  # kit USB board (unused); kit stands are kept
CHASSIS_L = np.array([-38.98, 0.39, 62.0])  # frame L origin in still_chassis_assm (plate top)
CAD = os.path.normpath(os.path.join(os.path.dirname(__file__), "../.."))
asm = lambda e: f"/assemblies/d/{D}/w/{W}/e/{e}"
state = json.load(open("placed.json")) if os.path.exists("placed.json") else {}
def save(): json.dump(state, open("placed.json", "w"), indent=1)
def T(t, R=np.eye(3)): M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M
def to_m(Hmm): Hm = Hmm.copy(); Hm[:3, 3] /= 1000; return Hm.flatten().tolist()
def set_xf(e, paths, Hmm, relative=False):
    # absolute on a sub-assembly moves every leaf to Hmm (collapses it); use relative on fresh (identity) inserts
    api.post(asm(e) + "/occurrencetransforms", json={
        "occurrences": [{"path": p} for p in paths], "transform": to_m(Hmm), "isRelative": relative})
def insert(e, body):  # returns new top-level instances
    ids0 = {i["id"] for i in api.get(asm(e)).json()["rootAssembly"]["instances"]}
    api.post(asm(e) + "/instances", json={"documentId": D, **body})
    return [i for i in api.get(asm(e)).json()["rootAssembly"]["instances"] if i["id"] not in ids0]

L2H = T(t_shell - [0, 0, 1.0])  # frame L -> housing assembly
cur = {o["path"][0]: np.array(o["transform"]).reshape(4, 4) for o in
       json.load(open("housing_asm.json"))["rootAssembly"]["occurrences"] if len(o["path"]) == 1}
pcb_now = cur[PCB_INST].copy(); pcb_now[:3, 3] *= 1000
X = {"pcb": np.load("D_pcb_snapped.npy") @ pcb_now, "orin": np.load("D_orin.npy"), "housing": L2H,
     "a1": L2H @ np.load("T_a1_L.npy"), "bridge": L2H @ np.load("T_bridge_L.npy"),
     "chassis": T(CHASSIS_L) @ np.linalg.inv(L2H)}
def main():
    if "--dry" in sys.argv:
        for k, v in X.items(): print(k, "\n", np.round(v, 3))
        return

    if "orin" not in state:
        state["orin"] = insert(HOUSING, {"elementId": ORIN, "isAssembly": True})[0]["id"]; save()
        set_xf(HOUSING, [[PCB_INST]], X["pcb"]); set_xf(HOUSING, [[state["orin"]]], X["orin"], relative=True); save()
    if "housing_asm_eid" not in state:  # import cover/base/standoffs; STEP assembly -> Onshape assembly tab
        url = f"https://cad.onshape.com/documents/{D}/w/{W}/e/{HOUSING}"
        subprocess.run([sys.executable, os.path.join(CAD, "tools/onshape_step_sync.py"), "push", url,
                        os.path.join(CAD, "out/orin_housing.step")], check=True)
        rec = next(iter(v for k, v in json.load(open(os.path.join(CAD, "out/.onshape_imports.json"))).items()
                        if k.endswith("orin_housing.step")))
        state["housing_asm_eid"] = next(e["id"] for e in rec["resultElements"] if e["type"] == "ASSEMBLY"); save()
    if "housing" not in state:
        state["housing"] = insert(HOUSING, {"elementId": state["housing_asm_eid"], "isAssembly": True})[0]["id"]
        set_xf(HOUSING, [[state["housing"]]], X["housing"], relative=True); save()
    for key, eid in (("a1", A1), ("bridge", BRIDGE)):
        if key in state: continue
        new = insert(HOUSING, {"elementId": eid, "isWholePartStudio": True})
        drop = [i["id"] for i in new if key == "a1" and i["name"].startswith(A1_DROP)]
        for nid in drop: api.delete(asm(HOUSING) + f"/instance/nodeid/{nid}")
        keep = [i["id"] for i in new if i["id"] not in drop]
        set_xf(HOUSING, [[i] for i in keep], X[key]); state[key] = keep; save()
    if "chassis" not in state and "--chassis" in sys.argv:
        state["chassis"] = insert(CHASSIS, {"elementId": HOUSING, "isAssembly": True})[0]["id"]
        set_xf(CHASSIS, [[state["chassis"]]], X["chassis"], relative=True); save()
    print(json.dumps(state, indent=1))


if __name__ == "__main__":
    main()
