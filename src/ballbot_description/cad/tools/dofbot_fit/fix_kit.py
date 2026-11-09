import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Full A1 dev-kit stack: re-add devboard + 7-pin connector, all kit parts at kit positions, stand feet on pads. ~9 calls."""
import json, math, numpy as np
from bb import api, D
from place import asm, HOUSING, A1, T, set_xf, state, save, L2H
SO = 3.0 + 24.6  # pad + kit stack (15 stand, 1.6 board, 8 stand) flange -> stand feet
def Hm(R, t): M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M
Rz = lambda a: np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
Ta = Hm(Rz(math.pi), [-17.0, 0.0, 78.0 + SO]) @ Hm(np.eye(3), [90.55, 132.92, -744.29])
np.save("T_a1_L.npy", Ta); Xa = L2H @ Ta
rep = state["repair"]
if "kit_full" not in rep:
    names = {i["id"]: i["name"] for i in api.get(asm(HOUSING)).json()["rootAssembly"]["instances"]}
    ids0 = set(names)
    have = {n.split(" <")[0] for n in names.values()}
    parts = api.get(f"/parts/d/{D}/w/{state.get('W', '9dc077545bceb395c1a5f5aa')}/e/{A1}").json()
    add = [p["partId"] for p in parts if p["name"].startswith(("devboard", "connector-7pin")) and p["name"] not in have]
    for pid in add:
        api.post(asm(HOUSING) + "/instances", json={"documentId": D, "elementId": A1, "partId": pid})
    new = [i["id"] for i in api.get(asm(HOUSING)).json()["rootAssembly"]["instances"] if i["id"] not in ids0]
    kit = state["a1"] + rep["stands"] + new  # every kit part shares one transform (part-studio positions)
    set_xf(HOUSING, [[i] for i in kit], Xa)
    state["a1"] = state["a1"] + new; rep["kit_full"] = True; save()
    print("added", len(new), "| kit parts placed:", len(kit))
