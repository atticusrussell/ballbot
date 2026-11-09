import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
from build123d import *
from OCP.BRepAdaptor import BRepAdaptor_Surface
import pickle
c = import_step("chassis.step")
pa = next(x for x in c.children if x.label.startswith("plate_assm"))
for ch in pa.children:
    bb = ch.bounding_box(); print(f"{ch.label!r:40} size=({bb.size.X:.1f},{bb.size.Y:.1f},{bb.size.Z:.1f}) min=({bb.min.X:.1f},{bb.min.Y:.1f},{bb.min.Z:.1f})")
ea = next(x for x in c.children if x.label == "EA25-5"); print("EA25-5", ea.bounding_box())
# store solids for later
pickle.dump([(x.label, x) for x in c.children], open("chassis_children.pkl", "wb"))
