import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Part Studio features on the imported housing base (orin_housing (1)) = orin_housing.board_bosses(). 7 calls.

Floor bosses + through M2.5 insert holes for the 6 board standoffs, and plate-screw head clearance above the floor.
Never re-push the import (re-ids parts, breaks mates). Resumable via placed.json ('bosses').
"""
import sys
from bb import api, D
from place import W, CAD, state, save
sys.path.insert(0, CAD + "/parts")
from orin_housing import BOARD_HOLES, PLATE_SCREWS, BOSS_D, BOSS_TOP, Z_FLOOR, M25_INS_D
PS, BASE_PART = "e2f571e82f0544a630201ca9", "JFH"
url = f"/partstudios/d/{D}/w/{W}/e/{PS}/features"
done = state.setdefault("bosses", {})

def q(qs): return {"btType": "BTMIndividualQuery-138", "queryString": qs}
def qty(pid, v): return {"btType": "BTMParameterQuantity-147", "parameterId": pid, "expression": v}
def enum(pid, name, v): return {"btType": "BTMParameterEnum-145", "parameterId": pid, "enumName": name, "value": v}
def add(name, f):
    if name in done: return done[name]
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
extrude("board boss screw clear", s3, "REMOVE", BOSS_TOP - Z_FLOOR + 0.5)
print(done)
