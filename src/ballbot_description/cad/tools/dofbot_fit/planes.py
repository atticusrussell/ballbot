import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
s = TopoDS_Shape(); BRepTools.Read_s(s, "region.brep", BRep_Builder())
ex = TopExp_Explorer(s, TopAbs_FACE); rows = []
while ex.More():
    f = TopoDS.Face_s(ex.Current()); a = BRepAdaptor_Surface(f)
    if a.GetType() == GeomAbs_Plane:
        g = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, g); A = g.Mass()
        if A > 800:
            n = a.Plane().Axis().Direction(); c = g.CentreOfMass(); b = Bnd_Box(); BRepBndLib.Add_s(f, b)
            rows.append((A, (round(n.X(), 3), round(n.Y(), 3), round(n.Z(), 3)), (round(c.X(), 1), round(c.Y(), 1), round(c.Z(), 1)), [round(v, 1) for v in b.Get()]))
    ex.Next()
for r in sorted(rows, key=lambda r: (r[1], r[2][1])): print(f"A={r[0]:8.0f} n={r[1]} c={r[2]} bb={r[3]}")
