import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import time
from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound, TopoDS
from OCP.BRepTools import BRepTools
from build123d import import_step
X = "/home/atticus/Downloads/cad/dofbot-pro/extracted/"
for n in ("orin_enclosure_assembly-frame.step", "orin_enclosure_zup.step"):
    print(n, import_step(X + n).bounding_box())
t = time.time()
r = STEPControl_Reader(); r.ReadFile("/home/atticus/Downloads/cad/dofbot-pro/DOFBOT-Pro-AI.STEP"); r.TransferRoots()
shape = r.OneShape(); print("read", round(time.time() - t), "s")
fr = import_step(X + "orin_enclosure_assembly-frame.step").bounding_box()
m = 40
lo = (fr.min.X - m, fr.min.Y - m, fr.min.Z - m); hi = (fr.max.X + m, fr.max.Y + m, fr.max.Z + m)
comp = TopoDS_Compound(); bld = BRep_Builder(); bld.MakeCompound(comp)
ex = TopExp_Explorer(shape, TopAbs_FACE); n = k = 0
while ex.More():
    f = ex.Current(); b = Bnd_Box(); BRepBndLib.Add_s(f, b); x0, y0, z0, x1, y1, z1 = b.Get()
    n += 1
    if x0 >= lo[0] and y0 >= lo[1] and z0 >= lo[2] and x1 <= hi[0] and y1 <= hi[1] and z1 <= hi[2]:
        bld.Add(comp, f); k += 1
    ex.Next()
print("faces", n, "kept", k)
BRepTools.Write_s(comp, "region.brep")
