import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Insert the A1_UART-Bridge composite part (as v1 still_chassis_assm uses) at the bridge pose. ~5 API calls.

The 111 loose bridge bodies (state["bridge"]) are deleted in the UI (one API call per instance otherwise).
"""
from bb import api, D
from place import W, HOUSING, BRIDGE, X, state, save, insert, set_xf

parts = api.get(f"/parts/d/{D}/w/{W}/e/{BRIDGE}").json()
comp = [p for p in parts if p.get("bodyType") == "composite"]
print([(p["name"], p["partId"]) for p in comp])
assert len(comp) == 1, "expected one composite part"
if "bridge_comp" not in state:
    new = insert(HOUSING, {"elementId": BRIDGE, "partId": comp[0]["partId"]})
    assert len(new) == 1, new
    state["bridge_comp"] = new[0]["id"]; save()
    set_xf(HOUSING, [[state["bridge_comp"]]], X["bridge"])  # absolute is safe on a top-level part
print("bridge composite instance:", state["bridge_comp"])
