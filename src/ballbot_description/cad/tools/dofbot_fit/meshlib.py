import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import numpy as np
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
def tris(shape, defl=0.5):
    BRepMesh_IncrementalMesh(shape, defl, False, 0.5, True)
    out = []; fid = []
    ex = TopExp_Explorer(shape, TopAbs_FACE); i = 0
    while ex.More():
        f = TopoDS.Face_s(ex.Current()); loc = TopLoc_Location()
        t = BRep_Tool.Triangulation_s(f, loc)
        if t is not None:
            tr = loc.Transformation()
            P = np.array([[*t.Node(k).Transformed(tr).Coord()] for k in range(1, t.NbNodes() + 1)])
            for k in range(1, t.NbTriangles() + 1):
                a, b, c = t.Triangle(k).Get(); out.append(P[[a - 1, b - 1, c - 1]]); fid.append(i)
        i += 1; ex.Next()
    return np.array(out), np.array(fid)
