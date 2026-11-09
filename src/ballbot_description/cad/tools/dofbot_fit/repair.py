import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Repair the 2026-11-09 placement (absolute transforms on sub-assemblies collapse their internals).

Re-pushes out/orin_housing.step as a fresh import (baked geometry), re-inserts Orin / housing / chassis
instances with RELATIVE transforms, lowers the A1 onto the dev-kit stands, adds the stands, groups each
assembly. Resumable via placed.json ('repair' dict). ~45 API calls.
"""
import json, os, subprocess, sys, numpy as np
from bb import api, D
from place import (W, HOUSING, CHASSIS, ORIN, A1, CAD, asm, insert, set_xf, X, T, state, save)
OLD_TABS = ["de65e6fb0906ebb44b25af41", "70f87f87c3f2a4885f779121", "20316124d56f673719dfa663",
            "ba7f96ab6f12a22bf0f3a52e"]  # first orin_housing import (blob, asm, parts, BOM)
DZ_A1 = (3.0 + 23.0) - 30.0  # pad + kit stands replaces the 30 mm standoffs
rep = state.setdefault("repair", {})
def gone(e, ids):
    for i in ids: api.delete(asm(e) + f"/instance/nodeid/{i}")
def group(e, paths, name):
    api.post(asm(e) + "/features", json={"feature": {
        "btType": "BTMMateGroup-65", "featureType": "mateGroup", "name": name, "parameters": [
            {"btType": "BTMParameterQueryWithOccurrenceList-67", "parameterId": "occurrencesQuery",
             "queries": [{"btType": "BTMIndividualOccurrenceQuery-626", "path": p} for p in paths]}]}})

RUN = __name__ == "__main__"
if RUN and "housing_ps_eid" not in rep:
    step = os.path.join(CAD, "out/orin_housing.step")
    subprocess.run([sys.executable, os.path.join(CAD, "tools/onshape_step_sync.py"), "push", "--new",
                    f"https://cad.onshape.com/documents/{D}/w/{W}/e/{HOUSING}", step], check=True)
    rec = next(v for k, v in json.load(open(os.path.join(CAD, "out/.onshape_imports.json"))).items()
               if k.endswith("orin_housing.step"))
    rep["housing_ps_eid"] = next(e["id"] for e in rec["resultElements"] if e["type"] == "PARTSTUDIO"); save()
if RUN and "deleted_old" not in rep:
    gone(HOUSING, [state["orin"], state["housing"]]); gone(CHASSIS, [state["chassis"]])
    for e in OLD_TABS: api.delete(f"/elements/d/{D}/w/{W}/e/{e}")
    rep["deleted_old"] = True; save()
if RUN and "orin" not in rep:
    rep["orin"] = insert(HOUSING, {"elementId": ORIN, "isAssembly": True})[0]["id"]
    set_xf(HOUSING, [[rep["orin"]]], X["orin"], relative=True); save()
if RUN and "housing" not in rep:
    new = insert(HOUSING, {"elementId": rep["housing_ps_eid"], "isWholePartStudio": True})  # flat parts, baked
    rep["housing"] = [i["id"] for i in new]; set_xf(HOUSING, [[i] for i in rep["housing"]], X["housing"]); save()
if RUN and "a1_lowered" not in rep:
    set_xf(HOUSING, [[i] for i in state["a1"]], T([0, 0, DZ_A1]), relative=True)
    rep["a1_lowered"] = True; save()
if RUN and "stands" not in rep:  # dev-kit stands from the A1 part studio, in A1 coordinates
    parts = api.get(f"/parts/d/{D}/w/{W}/e/{A1}").json()
    pids = [p["partId"] for p in parts if p["name"].startswith("stand")]
    ids0 = {i["id"] for i in api.get(asm(HOUSING)).json()["rootAssembly"]["instances"]}
    for pid in pids:
        api.post(asm(HOUSING) + "/instances", json={"documentId": D, "elementId": A1, "partId": pid})
    new = [i for i in api.get(asm(HOUSING)).json()["rootAssembly"]["instances"] if i["id"] not in ids0]
    s15 = [[i["id"]] for i in new if not i["name"].startswith("stand-8")]
    s8 = [[i["id"]] for i in new if i["name"].startswith("stand-8")]  # kit USB board (1.6) removed: close the gap
    set_xf(HOUSING, s15, X["a1"]); set_xf(HOUSING, s8, T([0, 0, 1.6]) @ X["a1"])  # T_a1_L already at SO=26
    rep["stands"] = [i["id"] for i in new]; save()
if RUN and "group_housing" not in rep:
    paths = [["MVWER4al7DjyElyq2"], [rep["orin"]]] + \
            [[i] for i in rep["housing"] + state["a1"] + rep["stands"] + state["bridge"]]
    group(HOUSING, paths, "electronics_group"); rep["group_housing"] = True; save()
if RUN and "chassis" not in rep:
    rep["chassis"] = insert(CHASSIS, {"elementId": HOUSING, "isAssembly": True})[0]["id"]
    set_xf(CHASSIS, [[rep["chassis"]]], X["chassis"], relative=True); save()
if RUN: print(json.dumps(rep, indent=1))
