import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""v2 housing stack in dofbot_pcb_housing: doc standoffs + new cover mated, offset mates raised by RAISE. ~50 calls.

Before: board_bosses.py; push out/orin_cover.step --new; in the UI delete the old cover, the home-made standoffs and
the lidar head, insert orin_cover + v1 standoffs (6 bn_3318 M2.5x10, 8 hex_stand_off_m2_5x20, 4 bn_3318 M2.5x5).
Standoff axis is part +X (stud end), so targets are Ry(90) (stud down). Resumable via placed.json ('v2').
"""
import json, re, sys, numpy as np
from urllib.parse import quote
from bb import api
from place import HOUSING, CAD, X, T, asm, state, save
import mates as m
sys.path.insert(0, CAD + "/parts")
from orin_housing import RAISE, BOARD_HOLES, ORIN_HOLES, LIDAR_HOLES, PCB_Z, PCB_T, Z_TOP, LIDAR_PAD

v2 = state.setdefault("v2", {}); done = m.done
RY = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)  # part +X -> -Z
if "ids" not in v2:
    inst = api.get(asm(HOUSING) + "?includeNonSolids=true").json()["rootAssembly"]["instances"]
    by = lambda pat: [i["id"] for i in sorted((i for i in inst if re.match(pat, i["name"])),
                                              key=lambda i: int(i["name"].rsplit("<", 1)[1][:-1]))]
    v2["ids"] = {"cover": by(r"orin_housing_cover")[0], "x10": by(r"BN 3318 3664350"), "x20": by(r"hex stand-off M2.5x20"),
                 "x5": by(r"BN 3318 3664347")}
    assert [len(v2["ids"][k]) for k in ("x10", "x20", "x5")] == [6, 8, 4], v2["ids"]; save()
ids = v2["ids"]

# 1. raise the existing offset targets on the base by RAISE (translation is in the flipped frame when flipPrimary)
RAISED = ["mc pcb target", "mc orin target", "mc bridge target", "mc lidar base target"] + \
         [f"mc bridge spacer {k} target" for k in range(1, 5)]
if not all(n in v2.get("raised", []) for n in RAISED):
    feats = {f["name"]: f for f in api.get(asm(HOUSING) + "/features").json()["features"]}
    for n in RAISED:
        if n in v2.setdefault("raised", []): continue
        f = feats[n]; P = {p["parameterId"]: p for p in f["parameters"]}
        flip = P.get("flipPrimary", {}).get("value", False)
        z = P["translationZ"]; old = float(z["expression"].split()[0]); assert z["expression"].endswith(" mm"), z
        z["expression"] = f"{old + (-RAISE if flip else RAISE):.6f} mm"
        r = api.post(asm(HOUSING) + "/features/featureid/" + quote(f["featureId"], safe=""), json={"feature": f}).json()
        st = (r.get("featureState") or {}).get("featureStatus"); print(f"  {n:<28} z {old} -> {z['expression']} {st}")
        if st != "OK": sys.exit(n + " update failed")
        v2["raised"].append(n); save()

# 2. cover (new import, frame L origin) origin-to-origin on the base
Wi = np.linalg.inv(m.mm(m.H[(m.BASE,)])); XL = Wi @ X["housing"]  # base part frame <- L
if "fasten cover v2" not in done:
    a = m.mc(HOUSING, "mc cover v2", (ids["cover"],))
    done["fasten cover v2"] = m.add(HOUSING, m.mate("fasten cover v2", m.fq(a), m.fq(done[f"mc origin {m.BASE}"]))); save()

# 3. standoffs: (name, instance, (x, y), z of part origin) in L
def conn(name, path, M):  # mate connector at a part origin, moved by M = T(t) @ Ry(90)
    t = M[:3, 3]; q = m.q
    return {"btType": "BTMMateConnector-66", "featureType": "mateConnector", "name": name, "parameters": [
        {"btType": "BTMParameterEnum-145", "parameterId": "originType", "enumName": "Origin type", "value": "ON_ENTITY"},
        {"btType": "BTMParameterQueryWithOccurrenceList-67", "parameterId": "originQuery", "queries": [m.origin(path)]},
        {"btType": "BTMParameterBoolean-144", "parameterId": "transform", "value": True},
        q("translationX", f"{t[0]:.6f} mm"), q("translationY", f"{t[1]:.6f} mm"), q("translationZ", f"{t[2]:.6f} mm"),
        {"btType": "BTMParameterEnum-145", "parameterId": "rotationType", "enumName": "Rotation axis", "value": "ABOUT_Y"},
        q("rotation", "90 deg")]}
z_orin = PCB_Z + PCB_T + 10  # 20 mm body centred on its origin, bottom on the PCB top
z_l20, z_l5 = Z_TOP + LIDAR_PAD + 10, Z_TOP + LIDAR_PAD + 25  # x20 centre; x5 origin = female (top) end
plan = [(f"board standoff {k + 1}", i, h, PCB_Z) for k, (i, h) in enumerate(zip(ids["x10"], BOARD_HOLES))]
plan += [(f"orin standoff {k + 1}", i, h, z_orin) for k, (i, h) in enumerate(zip(ids["x20"][:4], ORIN_HOLES))]
plan += [(f"lidar standoff20 {k + 1}", i, h, z_l20) for k, (i, h) in enumerate(zip(ids["x20"][4:], LIDAR_HOLES))]
plan += [(f"lidar standoff5 {k + 1}", i, h, z_l5) for k, (i, h) in enumerate(zip(ids["x5"], LIDAR_HOLES))]
v2["standoffs"] = {n: [i, list(h), z] for n, i, h, z in plan}; save()
for n, i, (x, y), z in plan:
    if f"fasten {n}" in done: continue
    if f"mc {n} target" not in done:
        done[f"mc {n} target"] = m.add(HOUSING, conn(f"mc {n} target", (m.BASE,), XL @ T([x, y, z], RY))); save()
    f = m.mate(f"fasten {n}", m.fq("IMC" + str(len(done))), m.fq(done[f"mc {n} target"]))  # implicit origin connector
    imc = conn("Mate connector", (i,), np.eye(4)); imc["parameters"] = imc["parameters"][:2]
    f["mateConnectors"] = [{**imc, "implicit": True, "isHidden": True, "featureId": "IMC" + str(len(done))}]
    done[f"fasten {n}"] = m.add(HOUSING, f); save()
    if "--one" in sys.argv: break
