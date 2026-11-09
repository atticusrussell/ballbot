import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
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
h = import_step("housing.step")
pcb = next(c for c in h.children if c.label.startswith("YB-EVV01"))
board = next(c for c in pcb.children if c.label == "Board_step")
print("PCB board bbox", board.bounding_box()); print("PCB holes (housing coords):", holes(board))
o = import_step("orin_assm.step")
carrier = next(c for c in next(c for c in o.children if c.label.startswith("600-")).children if c.label.startswith("600-"))
print("carrier bbox", carrier.bounding_box()); print("carrier holes:", [x for x in holes(carrier) if x[0] > 2.4])
