import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Lidar head lives in robot_assm (top level, v1's 'RPLidar A1 Head' part); its 'dof_lidar' revolute (URDF joint) goes to
the housing's lidar base.ipt leaf. base.ipt's origin is on the spin axis (Z up), the head part is in A1 kit coords,
so head connector = head origin moved to that axis point. 3 calls (+2 cached GETs). Head unsuppressed in the UI.
"""
import json, os, sys, numpy as np
from urllib.parse import quote
from bb import api
from place import asm, T, state, save
import mates as m
ROBOT, HEAD = "83ac8f3e1b8a64890594cf88", "MhnNZDbWRF/8T/NJv"
BASE_LEAF = ("MwVYb3BSH0u/35hbh", "MBDflk3MlGOi+TOXy", "MplDbrWUB4Ys3Lj6B", "MZBeORmRkwRTMk157")  # chassis/housing/base/base.ipt
AXIS = (-90.5533, -132.9176, 748.2923)  # base.ipt origin in A1 kit coords (a1_base_asm.json)
if not os.path.exists("robot_now.json"):
    json.dump(api.get(asm(ROBOT) + "?includeNonSolids=true").json(), open("robot_now.json", "w"))
    json.dump(api.get(asm(ROBOT) + "/features").json(), open("robot_features.json", "w"))
rh = state.setdefault("robot_head", {})
def mc(key, path, M=np.eye(4)):
    if key not in rh: rh[key] = m.add(ROBOT, m.connector(key, path, M)); save()
    return rh[key]
a, b = mc("mc lidar head joint", (HEAD,), T(AXIS)), mc("mc lidar base joint", BASE_LEAF)
if "dof_lidar" not in rh:
    f = next(x for x in json.load(open("robot_features.json"))["features"] if x["name"] == "dof_lidar")
    f.pop("mateConnectors", None)
    next(p for p in f["parameters"] if p["parameterId"] == "primaryAxisAlignment")["value"] = False  # v1 flip, not needed
    next(p for p in f["parameters"] if p["parameterId"] == "mateConnectorsQuery")["queries"] = [m.fq(a), m.fq(b)]
    r = api.post(asm(ROBOT) + "/features/featureid/" + quote(f["featureId"], safe=""), json={"feature": f}).json()
    st = (r.get("featureState") or {}).get("featureStatus"); print("dof_lidar", st)
    if st != "OK": sys.exit(f"dof_lidar update failed: {r.get('featureState')}")
    rh["dof_lidar"] = f["featureId"]; save()
