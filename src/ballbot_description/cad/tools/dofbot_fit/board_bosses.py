import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Part Studio features on the imported housing base (orin_housing (1)) = orin_housing.board_bosses(). 7 calls.

Floor bosses + through M2.5 insert holes for the 6 board standoffs, and plate-screw head clearance above the floor.
Never re-push the import (re-ids parts, breaks mates). Resumable via placed.json ('bosses').
"""
import sys
from bb import api, D
from place import W, CAD, state, save
sys.path.insert(0, CAD + "/parts")
from orin_housing import BOARD_HOLES, PLATE_SCREWS, BOSS_D, BOSS_TOP, Z_FLOOR, M25_INS_D, BLOCK_Z
PS, BASE_PART = "e2f571e82f0544a630201ca9", "JFH"
url = f"/partstudios/d/{D}/w/{W}/e/{PS}/features"
done = state.setdefault("bosses", {})

def q(qs): return {"btType": "BTMIndividualQuery-138", "queryString": qs}
def qty(pid, v): return {"btType": "BTMParameterQuantity-147", "parameterId": pid, "expression": v}
def enum(pid, name, v): return {"btType": "BTMParameterEnum-145", "parameterId": pid, "enumName": name, "value": v}
def add(name, f):
    if name in done and name not in sys.argv: return done[name]
    if name in done:  # --update: re-post with the same id
        f["name"] = name; f["featureId"] = done[name]
        r = api.post(url + "/featureid/" + done[name], json={"feature": f}).json()
        print(name, "updated", (r.get("featureState") or {}).get("featureStatus")); return done[name]
    f["name"] = name
    r = api.post(url, json={"feature": f}).json()
    st = (r.get("featureState") or {}).get("featureStatus"); fid = r["feature"]["featureId"]; print(f"{name:<28} {fid} {st}")
    if st != "OK": sys.exit(f"{name} failed: {r.get('featureState')}")
    done[name] = fid; save(); return fid
def sketch(name, plane_q, circles):  # circles: (x, y, d) in mm
    ents = [{"btType": "BTMSketchCurve-4", "entityId": f"c{i}", "geometry": {
        "btType": "BTCurveGeometryCircle-115", "radius": d / 2000, "xCenter": x / 1000, "yCenter": y / 1000,
        "xDir": 1.0, "yDir": 0.0, "clockwise": False}} for i, (x, y, d) in enumerate(circles)]
    return add(name, {"btType": "BTMSketch-151", "featureType": "newSketch", "entities": ents, "constraints": [],
                      "parameters": [{"btType": "BTMParameterQueryList-148", "parameterId": "sketchPlane", "queries": [plane_q]}]})
def extrude(name, sk, op, depth):
    return add(name, {"btType": "BTMFeature-134", "featureType": "extrude", "parameters": [
        enum("bodyType", "ExtendedToolBodyType", "SOLID"), enum("operationType", "NewBodyOperationType", op),
        {"btType": "BTMParameterQueryList-148", "parameterId": "entities", "queries": [
            q(f'query = qSketchRegion(id + "{sk}", false);')]},
        enum("endBound", "BoundingType", "BLIND"), qty("depth", f"{depth} mm"),
        {"btType": "BTMParameterBoolean-144", "parameterId": "defaultScope", "value": False},
        {"btType": "BTMParameterQueryList-148", "parameterId": "booleanScope", "queries": [
            {"btType": "BTMIndividualQuery-138", "deterministicIds": [BASE_PART]}]}]})

top = q('query = qCreatedBy(makeId("Top"), EntityType.FACE);')
pl = add("board boss floor plane", {"btType": "BTMFeature-134", "featureType": "cPlane", "parameters": [
    {"btType": "BTMParameterQueryList-148", "parameterId": "entities", "queries": [top]},
    enum("cplaneType", "CPlaneType", "OFFSET"), qty("offset", f"{Z_FLOOR} mm")]})
on_pl = q(f'query = qCreatedBy(makeId("{pl}"), EntityType.FACE);')
s1 = sketch("board boss sketch", on_pl, [(x, y, BOSS_D) for x, y in BOARD_HOLES])
extrude("board boss", s1, "ADD", BOSS_TOP - Z_FLOOR)
s2 = sketch("board boss insert sketch", top, [(x, y, M25_INS_D) for x, y in BOARD_HOLES])
extrude("board boss insert", s2, "REMOVE", BOSS_TOP)
s3 = sketch("board boss screw clear sketch", on_pl, [(x, y, 6.5) for x, y in PLATE_SCREWS])
extrude("board boss screw clear", s3, "REMOVE", BLOCK_Z - Z_FLOOR + 0.5)  # head + driver clear past the corner blocks
print(done)

# v2 cleanup: fill the obsolete corner-block pin pockets and legacy M3 PCB inserts (board is 5 mm above the blocks)
from orin_housing import PIN_RELIEF, PIN_CLEAR, PCB_HOLES, HALF_L, W_OUT, T, GAP, M3_INS_D, BLOCK_Z
def clip(x0, x1, y0, y1):  # pocket box clipped to its corner block footprint
    bx = (54.0, HALF_L) if x0 > 0 else (-HALF_L, -54.0); by = (44.0, W_OUT - T - GAP) if y0 > 0 else (-(W_OUT - T - GAP), -44.0)
    return max(x0, bx[0]), min(x1, bx[1]), max(y0, by[0]), min(y1, by[1])
def rect(i, x0, x1, y0, y1):
    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [{"btType": "BTMSketchCurveSegment-155", "entityId": f"r{i}.{k}", "startPointId": f"r{i}.{k}.s",
             "endPointId": f"r{i}.{k}.e", "startParam": 0.0, "endParam": float(np.hypot(b[0] - a[0], b[1] - a[1])) / 1000,
             "geometry": {"btType": "BTCurveGeometryLine-117", "pntX": a[0] / 1000, "pntY": a[1] / 1000,
                          "dirX": float(np.sign(b[0] - a[0])), "dirY": float(np.sign(b[1] - a[1]))}}
            for k, (a, b) in enumerate(zip(pts, pts[1:] + pts[:1]))]
import numpy as np
if "board fill" not in done:
    ents = [e for i, (x0, x1, y0, y1) in enumerate(PIN_RELIEF)
            for e in rect(i, *clip(x0 - PIN_CLEAR, x1 + PIN_CLEAR, y0 - PIN_CLEAR, y1 + PIN_CLEAR))]
    ents += [{"btType": "BTMSketchCurve-4", "entityId": f"h{i}", "geometry": {"btType": "BTCurveGeometryCircle-115",
              "radius": M3_INS_D / 2000, "xCenter": x / 1000, "yCenter": y / 1000, "xDir": 1.0, "yDir": 0.0,
              "clockwise": False}} for i, (x, y) in enumerate(PCB_HOLES)]
    s4 = add("board fill sketch", {"btType": "BTMSketch-151", "featureType": "newSketch", "entities": ents, "constraints": [],
             "parameters": [{"btType": "BTMParameterQueryList-148", "parameterId": "sketchPlane", "queries": [on_pl]}]})
    extrude("board fill", s4, "ADD", BLOCK_Z - Z_FLOOR)
