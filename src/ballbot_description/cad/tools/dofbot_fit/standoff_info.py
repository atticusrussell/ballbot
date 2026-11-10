import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Origin, length and thread/stud geometry of v1's standoff part studios (1 call each) -> standoffs_v1.json."""
import json
from bb import api, D
from place import W
PS = {"m25x20": "6558f1f1601bdf01569e5389", "m25x10_mf": "3f102943191b8e086f660e5a",
      "m25x10_ff": "1f51c46432a10cded301538f", "m25x5_ff": "5d6b4351a91ff3f16031108c"}
out = json.load(open("standoffs_v1.json")) if __import__("os").path.exists("standoffs_v1.json") else {}
for k, e in PS.items():
    if k in out: continue
    out[k] = api.get(f"/partstudios/d/{D}/w/{W}/e/{e}/bodydetails").json(); json.dump(out, open("standoffs_v1.json", "w"))
v = lambda d: [round(d[c] * 1000, 2) for c in "xyz"]
for k, r in out.items():
    for b in r["bodies"]:
        fs = b["faces"]; lo = [min(f["box"]["minCorner"][c] for f in fs) * 1000 for c in "xyz"]
        hi = [max(f["box"]["maxCorner"][c] for f in fs) * 1000 for c in "xyz"]
        print(k, b["id"], "bbox", [round(a, 2) for a in lo], [round(a, 2) for a in hi])
        for f in fs:
            s = f["surface"]
            if s["type"] == "CYLINDER":
                print("   cyl r", round(s["radius"] * 1000, 3), "axis", v(s["axis"]) if "axis" in s else "",
                      "z", round(f["box"]["minCorner"]["z"] * 1000, 2), round(f["box"]["maxCorner"]["z"] * 1000, 2))
