import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
# horizontal holes near the enclosure's bottom edge, and what sits below it
from OCP.BRepTools import BRepTools
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
s = TopoDS_Shape(); BRepTools.Read_s(s, "region.brep", BRep_Builder())
ex = TopExp_Explorer(s, TopAbs_FACE)
print("-- cylinders below y=-30 (any axis), r 1..4")
seen = set()
while ex.More():
    f = TopoDS.Face_s(ex.Current()); a = BRepAdaptor_Surface(f); b = Bnd_Box(); BRepBndLib.Add_s(f, b)
    x0, y0, z0, x1, y1, z1 = b.Get()
    if a.GetType() == GeomAbs_Cylinder and y1 < -30:
        c = a.Cylinder(); d = c.Axis().Direction(); l = c.Axis().Location()
        if 0.8 < c.Radius() < 4:
            key = (round(c.Radius(), 2), round(x0, 0), round(y0, 0), round(z0, 0))
            if key not in seen:
                seen.add(key); print(f"r={c.Radius():.2f} axis=({d.X():+.0f},{d.Y():+.0f},{d.Z():+.0f}) bb=({x0:.1f},{y0:.1f},{z0:.1f})..({x1:.1f},{y1:.1f},{z1:.1f})")
    if a.GetType() == GeomAbs_Plane and y1 < -38:
        g = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, g)
        if g.Mass() > 300:
            n = a.Plane().Axis().Direction()
            print(f"PLANE A={g.Mass():.0f} n=({n.X():+.1f},{n.Y():+.1f},{n.Z():+.1f}) bb=({x0:.1f},{y0:.1f},{z0:.1f})..({x1:.1f},{y1:.1f},{z1:.1f})")
    ex.Next()
