import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Native 'heat-set inserts' Part Studio (Standard Content has no plastic inserts). 5 calls, resumable ('hw').

M2 / M2.5 / M3 tubes sized to the printed holes (OD = hole Ø, length = hole depth, bore = tap minor Ø), at x = 0/10/20,
top face on z = 0, body down -Z. Mate with an implicit connector on the part origin moved by (x, 0, 0).
"""
import sys
from bb import api, D
from place import W, CAD, state, save
sys.path.insert(0, CAD + "/parts")
from orin_housing import M2_INS_D, M2_INS_L, M25_INS_D, M25_INS_L, M3_INS_D, M3_INS_L
INSERTS = {"M2": (0.0, M2_INS_D, M2_INS_L, 1.6), "M2.5": (10.0, M25_INS_D, M25_INS_L, 2.05), "M3": (20.0, M3_INS_D, M3_INS_L, 2.5)}
hw = state.setdefault("hw", {})
if __name__ == "__main__":
    if "ps" not in hw:
        hw["ps"] = api.post(f"/partstudios/d/{D}/w/{W}", json={"name": "heat-set inserts"}).json()["id"]; save()
    url = f"/partstudios/d/{D}/w/{W}/e/{hw['ps']}/features"
    def add(name, f):
        if name in hw: return hw[name]
        f["name"] = name; r = api.post(url, json={"feature": f}).json()
        st = (r.get("featureState") or {}).get("featureStatus"); print(name, r["feature"]["featureId"], st)
        if st != "OK": sys.exit(f"{name}: {r.get('featureState')}")
        hw[name] = r["feature"]["featureId"]; save(); return hw[name]
    circ = lambda i, x, d: {"btType": "BTMSketchCurve-4", "entityId": f"c{i}", "geometry": {"btType": "BTCurveGeometryCircle-115",
        "radius": d / 2000, "xCenter": x / 1000, "yCenter": 0.0, "xDir": 1.0, "yDir": 0.0, "clockwise": False}}
    ents = [c for i, (x, od, _, bore) in enumerate(INSERTS.values()) for c in (circ(2 * i, x, od), circ(2 * i + 1, x, bore))]
    sk = add("inserts sketch", {"btType": "BTMSketch-151", "featureType": "newSketch", "entities": ents, "constraints": [],
        "parameters": [{"btType": "BTMParameterQueryList-148", "parameterId": "sketchPlane", "queries": [
            {"btType": "BTMIndividualQuery-138", "queryString": 'query = qCreatedBy(makeId("Top"), EntityType.FACE);'}]}]})
    for n, (x, od, L, bore) in INSERTS.items():  # annulus only (inner disk filtered), down from z 0
        add(f"insert {n}", {"btType": "BTMFeature-134", "featureType": "extrude", "parameters": [
            {"btType": "BTMParameterEnum-145", "parameterId": "bodyType", "enumName": "ExtendedToolBodyType", "value": "SOLID"},
            {"btType": "BTMParameterEnum-145", "parameterId": "operationType", "enumName": "NewBodyOperationType", "value": "NEW"},
            {"btType": "BTMParameterQueryList-148", "parameterId": "entities", "queries": [{"btType": "BTMIndividualQuery-138",
                "queryString": f'query = qContainsPoint(qSketchRegion(id + "{sk}", true), vector({x + (od + bore) / 4}, 0, 0) * millimeter);'}]},
            {"btType": "BTMParameterEnum-145", "parameterId": "endBound", "enumName": "BoundingType", "value": "BLIND"},
            {"btType": "BTMParameterQuantity-147", "parameterId": "depth", "expression": f"{L} mm"},
            {"btType": "BTMParameterBoolean-144", "parameterId": "oppositeDirection", "value": True}]})
    print(hw)
