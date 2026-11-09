import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
# vertical (Y-axis) cylinders/holes in the fused region between PCB and shell top
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from collections import defaultdict
s = TopoDS_Shape(); BRepTools.Read_s(s, "region.brep", BRep_Builder())
ex = TopExp_Explorer(s, TopAbs_FACE); g = defaultdict(list)
while ex.More():
    f = TopoDS.Face_s(ex.Current()); a = BRepAdaptor_Surface(f)
    if a.GetType() == GeomAbs_Cylinder:
        c = a.Cylinder(); d = c.Axis().Direction(); l = c.Axis().Location()
        if abs(abs(d.Y()) - 1) < 1e-3 and 1.0 < c.Radius() < 3.5:
            b = Bnd_Box(); BRepBndLib.Add_s(f, b); x0, y0, z0, x1, y1, z1 = b.Get()
            g[(round(l.X(), 1), round(l.Z(), 1))].append((round(c.Radius(), 2), round(y0, 1), round(y1, 1)))
    ex.Next()
for k in sorted(g):
    ys = [v for v in g[k]]
    if -14.3 <= k[0] <= 115.7 and 232 <= k[1] <= 340: print(k, sorted(set(ys))[:6])
