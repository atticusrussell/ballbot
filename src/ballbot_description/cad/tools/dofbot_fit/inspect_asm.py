import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import sys, time
from build123d import import_step
t = time.time(); c = import_step(sys.argv[1]); print("load", round(time.time() - t), "s; label", c.label)
def walk(n, d=0):
    if d > int(sys.argv[2]): return
    bb = n.bounding_box()
    print("  " * d + f"{n.label!r:40} {type(n).__name__:9} size=({bb.size.X:.1f},{bb.size.Y:.1f},{bb.size.Z:.1f}) min=({bb.min.X:.1f},{bb.min.Y:.1f},{bb.min.Z:.1f}) n={len(n.children)}")
    for ch in sorted(n.children, key=lambda x: -x.bounding_box().diagonal)[:12]: walk(ch, d + 1)
walk(c)
