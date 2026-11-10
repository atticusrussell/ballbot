import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
"""Bounding boxes (part frame) of the hardware parts used in v2, 1 call each, cached in hw_frames.json."""
import json, os
from bb import api
from place import W
D = "833f9762904995f22ff7dd9f"
PAN = "JTQwc2NwPWpjY3EzN3R5MW5rcTBLa21OdU5SRE9aeGMzU1hEMVREYndnWkdxR2N6VXMlM0Q7QmFzaWNEaWFtZXRlcj0wLjAwMjUrbWV0ZXI7SGVhZERpYW1ldGVyPTAuMDA1K21ldGVyO0hlYWRIZWlnaHQ9MC4wMDIxMDAwMDAwMDAwMDAwMDAzK21ldGVyO0hlYWRSYWRpdXM9MC4wMDQrbWV0ZXI7TGVuZ3RoPTAuMDA2K21ldGVyO1BpdGNoPTQuNTAwMDAwMDAwMDAwMDAwNEUtNCttZXRlcjtSZWNlc3NDb3JlQW5nbGU9MC4xMjIxNzMwNDc2Mzk2MDMwNytyYWRpYW47UmVjZXNzQ29yZURpYW1ldGVyPTAuMDAxMzIrbWV0ZXI7UmVjZXNzRGVwdGg9MC4wMDE1K21ldGVyO1JlY2Vzc0xlbmd0aD0wLjAwMjYwMDAwMDAwMDAwMDAwMDMrbWV0ZXI7UmVjZXNzV2lkdGg9Ny40RS00K21ldGVyO1RocmVhZExlbmd0aD0wLjAwNittZXRlcjtUcmlhbmdsZUhlaWdodD0zLjg5NzExNDI5OTk5OTk5OTZFLTQrbWV0ZXI7VW5kZXJIZWFkRmlsbGV0PTEuMEUtNCttZXRlcg"
NUT = "JTQwc2NwPWRYWlpXclBaZE5ySVJFNzRhM05yZjhWYlF5c2JESll5QWVrUURvMW5FMnclM0Q7TWFqb3JEaWFtZXRlcj0wLjAwMjUrbWV0ZXI7TWlub3JEaWFtZXRlcj0wLjAwMjAxMyttZXRlcjtQaXRjaD00LjUwMDAwMDAwMDAwMDAwMDRFLTQrbWV0ZXI7VGhpY2tuZXNzPTAuMDAxNittZXRlcjtXaWR0aEFjcm9zc0ZsYXRzPTAuMDA1K21ldGVy"
PARTS = {  # name: (doc, wvm path, element, partId, configuration)
    "pan_m25x6": ("bb85f80711e95ef1833089a6", "v/0e9f21a2f560210eb1568737", "f291ce2ce48d4d493a84403b", "JHD", PAN),
    "nut_m25": ("563fdfbda4e30add44d190cc", "v/1b5c926bdbe733ba990b078a", "9be3bc199ebb0c96e1330d84", "JHD", NUT),
    "m2x6": (D, f"w/{W}", "d8ad0d58caaaa605fc7a6870", "JFf", None),
    "m3x8": (D, f"w/{W}", "90240809d158ea9f3c66b554", "JFH", None)}
out = json.load(open("hw_frames.json")) if os.path.exists("hw_frames.json") else {}
for k, (d, wv, e, pid, cfg) in PARTS.items():
    if k in out: continue
    r = api.get(f"/partstudios/d/{d}/{wv}/e/{e}/bodydetails" + (f"?configuration={cfg}" if cfg else "")).json()
    b = next(b for b in r["bodies"] if b["id"] == pid)
    lo = [min(f["box"]["minCorner"][c] for f in b["faces"]) * 1000 for c in "xyz"]
    hi = [max(f["box"]["maxCorner"][c] for f in b["faces"]) * 1000 for c in "xyz"]
    out[k] = {"lo": lo, "hi": hi}; json.dump(out, open("hw_frames.json", "w"), indent=1)
for k, v in out.items(): print(k, [round(a, 2) for a in v["lo"]], [round(a, 2) for a in v["hi"]])
