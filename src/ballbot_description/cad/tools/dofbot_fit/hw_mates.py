import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""v2 hardware + PCB-on-standoff mates, 1 call each: both connectors implicit inside the mate (no connector features).

Parts are inserted in the UI first (Standard Content ISO 7045 pan heads, thin nuts, 'heat-set inserts' Part Studio).
Pan head frame: origin on the head top, +Z toward the head (head height from the configuration). Thin nut: origin at
its centre. Inserts: origin on the part studio origin, part at x = 0/10/20, top face z 0, body -Z.
Resumable via placed.json ('hw'). --chassis does the base -> plate screws + nuts in still_chassis_assm.
"""
import base64, json, re, sys, numpy as np
from scipy.spatial.transform import Rotation as R
from bb import api
from place import HOUSING, CHASSIS, CAD, X, T, asm, state, save
import mates as m
sys.path.insert(0, CAD + "/parts")
from orin_housing import (BOARD_HOLES, ORIN_HOLES, LIDAR_HOLES, BRIDGE_HOLES, COVER_SCREWS, COVER_SCREW_Z, PLATE_SCREWS,
                          PCB_Z, BOSS_TOP, Z_TOP, LIDAR_PAD, W_OUT, T as WALL, GAP)
from hw_inserts_ps import INSERTS
hw = state.setdefault("hw", {}); done = m.done
SEAT = {"bridge": 85.003, "lidar": 114.5, "orin": 36.98}  # screw head seats in L (hw_heights.py)
RX = lambda a: R.from_euler("x", a, degrees=True).as_matrix()
RY90 = R.from_euler("y", 90, degrees=True).as_matrix()

def implicit(fid, path, M=np.eye(4)):  # implicit connector at a part origin moved by M (flip + one-axis rotation)
    M = M.copy(); flip = False
    for fl in (False, True):
        Mt = M.copy()
        if fl: F = np.diag([1, -1, -1]); Mt[:3, :3] = F @ Mt[:3, :3]; Mt[:3, 3] = F @ Mt[:3, 3]
        rv = R.from_matrix(Mt[:3, :3]).as_rotvec(degrees=True); ax = [i for i in range(3) if abs(rv[i]) > 1e-6]
        if len(ax) <= 1: flip, M = fl, Mt; break
    else: raise ValueError(f"{fid}: not a one-axis rotation {rv}")
    a = ax[0] if ax else 2; q = m.q; t = M[:3, 3]
    return {"btType": "BTMMateConnector-66", "featureType": "mateConnector", "name": "Mate connector", "implicit": True,
            "isHidden": True, "featureId": fid, "parameters": [
        {"btType": "BTMParameterEnum-145", "parameterId": "originType", "enumName": "Origin type", "value": "ON_ENTITY"},
        {"btType": "BTMParameterQueryWithOccurrenceList-67", "parameterId": "originQuery", "queries": [m.origin(path)]},
        *([{"btType": "BTMParameterBoolean-144", "parameterId": "flipPrimary", "value": True}] if flip else []),
        {"btType": "BTMParameterBoolean-144", "parameterId": "transform", "value": True},
        q("translationX", f"{t[0]:.6f} mm"), q("translationY", f"{t[1]:.6f} mm"), q("translationZ", f"{t[2]:.6f} mm"),
        {"btType": "BTMParameterEnum-145", "parameterId": "rotationType", "enumName": "Rotation axis", "value": "ABOUT_" + "XYZ"[a]},
        q("rotation", f"{rv[a]:.6f} deg")]}
PLAN = []
def fasten(e, name, child, parent):  # child/parent: (path, M)
    PLAN.append((name, child, parent))
    if name in done or "--plan-only" in sys.argv: return
    n = len(done); a, b = implicit(f"IA{n}", *child), implicit(f"IB{n}", *parent)
    f = m.mate(name, m.fq(a["featureId"]), m.fq(b["featureId"])); f["mateConnectors"] = [a, b]
    done[name] = m.add(e, f); save()
def cfg(i):  # decoded Standard Content configuration -> {param: mm}
    s = base64.b64decode(i["configuration"] + "==").decode(errors="ignore")
    return {k: float(v) * 1000 for k, v in re.findall(r"(\w+)=([-\d.E]+)\+meter", s)}

if "--chassis" not in sys.argv:
    inst = api.get(asm(HOUSING) + "?includeNonSolids=true").json()["rootAssembly"]["instances"] if "housing_hw" not in hw else None
    if inst is not None:
        pan = [i for i in inst if i["name"].startswith("PZ Pan head screw")]
        hw["housing_hw"] = {"pan": {d: [(i["id"], cfg(i)["HeadHeight"]) for i in pan if f"M{d}x" in i["name"]] for d in ("2", "2.5", "3")},
                            "ins": {k: [i["id"] for i in inst if i["name"].startswith(f"heat-set insert {k}x")] for k in INSERTS}}
        save()
    H = hw["housing_hw"]; print({k: len(v) for k, v in H["pan"].items()}, {k: len(v) for k, v in H["ins"].items()})
    XL = np.linalg.inv(m.mm(m.H[(m.BASE,)])) @ X["housing"]; base = (m.BASE,)
    # PCB: bottom of hole 1 onto board standoff 1's top (female end), world-aligned frames
    occ = {tuple(o["path"]): np.array(o["transform"]).reshape(4, 4) for o in json.load(open("housing_v2_now.json"))["rootAssembly"]["occurrences"]}
    leaf = (state["repair"]["pcb"], "MsqHB/2KD9gNBwaQx"); s1, h1, _ = state["v2"]["standoffs"]["board standoff 1"]
    Pb = m.mm(occ[leaf]) if leaf in occ else None
    if "fasten pcb v2b" not in done:  # superseded by v2b_remate.py (PCB re-inserted, turned 180)
        fasten(HOUSING, "fasten pcb on standoff", (leaf, np.linalg.inv(Pb) @ X["housing"] @ T([*h1, PCB_Z])),
               ((s1,), T([0, 0, 0], RY90.T)))
    if "--one" in sys.argv and "--plan-only" not in sys.argv: sys.exit()
    jobs = []  # (name, child path, child M, parent M in L)
    for g, holes, d in (("lidar", LIDAR_HOLES, "2.5"), ("orin", ORIN_HOLES, "2.5"), ("bridge", BRIDGE_HOLES, "2")):
        ids = H["pan"][d][:4] if g != "orin" else H["pan"]["2.5"][4:8]
        jobs += [(f"fasten {g} screw {k + 1}", i, np.eye(4), T([x, y, SEAT[g] + hh])) for k, ((i, hh), (x, y)) in enumerate(zip(ids, holes))]
    for k, ((i, hh), (x, s)) in enumerate(zip(H["pan"]["3"], COVER_SCREWS)):
        jobs.append((f"fasten cover screw {k + 1}", i, np.eye(4), T([x, s * (W_OUT + hh), COVER_SCREW_Z], RX(-90 * s))))
    tops = [("floor", BOARD_HOLES, "M2.5", BOSS_TOP), ("pad", LIDAR_HOLES, "M2.5", Z_TOP + LIDAR_PAD), ("bridge", BRIDGE_HOLES, "M2", Z_TOP)]
    used = {"M2.5": 0, "M2": 0, "M3": 0}
    for g, holes, k_, z in tops:
        for k, (x, y) in enumerate(holes):
            i = H["ins"][k_][used[k_]]; used[k_] += 1
            jobs.append((f"fasten {g} insert {k + 1}", i, T([INSERTS[k_][0], 0, 0]), T([x, y, z])))
    for k, (x, s) in enumerate(COVER_SCREWS):  # horizontal M3 inserts in the base blocks, top on the block face
        jobs.append((f"fasten cover insert {k + 1}", H["ins"]["M3"][k], T([INSERTS["M3"][0], 0, 0]),
                     T([x, s * (W_OUT - WALL - GAP), COVER_SCREW_Z], RX(-90 * s))))
    for n, i, Mc, ML in jobs: fasten(HOUSING, n, ((i,), Mc), (base, XL @ ML))
else:
    if "chassis_hw" not in hw:
        inst = api.get(asm(CHASSIS)).json()["rootAssembly"]["instances"]; old = set(hw.get("chassis_before", []))
        hw["chassis_hw"] = {"pan": [i["id"] for i in inst if i["name"].startswith("PZ Pan head screw M2.5") and i["id"] not in old],
                            "nut": [i["id"] for i in inst if i["name"].startswith("Hex thin nut grade A & B M2.5") and i["id"] not in old],
                            "hh": next(cfg(i)["HeadHeight"] for i in inst if i["name"].startswith("PZ Pan head screw M2.5"))}; save()
    C = hw["chassis_hw"]; print(len(C["pan"]), len(C["nut"]))
    base = (state["repair"]["chassis"], m.BASE); XL = np.linalg.inv(m.mm(m.H[(m.BASE,)])) @ X["housing"]
    for k, (x, y) in enumerate(PLATE_SCREWS):  # pan head on the counterbore floor (L z 1), nut under the 2 mm plate
        fasten(CHASSIS, f"fasten plate screw {k + 1}", ((C["pan"][k],), np.eye(4)), (base, XL @ T([x, y, 1.0 + C["hh"]])))
        fasten(CHASSIS, f"fasten plate nut {k + 1}", ((C["nut"][k],), np.eye(4)), (base, XL @ T([x, y, -2.8])))
