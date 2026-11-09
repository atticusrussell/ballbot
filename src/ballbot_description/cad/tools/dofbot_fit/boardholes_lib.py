from build123d import *
from OCP.BRepAdaptor import BRepAdaptor_Surface
def holes(solid):
    out = set()
    for f in solid.faces():
        if f.geom_type == GeomType.CYLINDER:
            cy = BRepAdaptor_Surface(f.wrapped).Cylinder(); l = cy.Axis().Location()
            if abs(abs(cy.Axis().Direction().Z()) - 1) < 1e-6 and cy.Radius() > 1.0:
                out.add((round(2 * cy.Radius(), 2), round(l.X(), 2), round(l.Y(), 2)))
    return sorted(out)
