import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Fix the YB-EVV01 PCB assembly itself: SOT-223 was stored ~43 mm off the board (earlier repairs only fixed it in
context). Ungroup, move it to its verified board-relative pose, regroup the same 1048 occurrences. 4 calls."""
import json, numpy as np
from urllib.parse import quote
from bb import api
from place import asm, set_xf
import mates as m
PCB = "fc16901070e8ac7eef2142d4"; B, S = "MsqHB/2KD9gNBwaQx", "MnzMARmxsEcjdhQY/"
occ = lambda f: {tuple(o["path"]): m.mm(np.array(o["transform"]).reshape(4, 4)) for o in json.load(open(f))["rootAssembly"]["occurrences"]}
O = occ("housing_v2_now.json"); old = json.load(open("placed.json"))["v2b"]["old"]["pcb"]
rel = np.linalg.inv(O[(old, B)]) @ O[(old, S)]  # verified SOT pose relative to the board
E_ = {tuple(o["path"]): m.mm(np.array(o["transform"]).reshape(4, 4)) for o in api.get(asm(PCB)).json()["rootAssembly"]["occurrences"]}
delta = E_[(B,)] @ rel @ np.linalg.inv(E_[(S,)])
g = json.load(open(f"feat_{PCB}.json"))["features"][0]; assert g["name"] == "Group 1"
api.delete(asm(PCB) + "/features/featureid/" + quote(g["featureId"], safe=""))
set_xf(PCB, [[S]], delta, relative=True)
paths = [q["path"] for p in g["parameters"] if "queries" in p for q in p["queries"]]
r = api.post(asm(PCB) + "/features", json={"feature": {"btType": "BTMMateGroup-65", "featureType": "mateGroup", "name": "Group 1",
    "parameters": [{"btType": "BTMParameterQueryWithOccurrenceList-67", "parameterId": "occurrencesQuery",
    "queries": [{"btType": "BTMIndividualOccurrenceQuery-626", "path": p} for p in paths]}]}}).json()
print("regrouped", len(paths), (r.get("featureState") or {}).get("featureStatus"))
