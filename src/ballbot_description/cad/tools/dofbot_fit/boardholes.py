import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
from build123d import *
from boardholes_lib import holes
h = import_step("housing.step")
pcb = next(c for c in h.children if c.label.startswith("YB-EVV01"))
board = next(c for c in pcb.children if c.label == "Board_step")
print("PCB board bbox", board.bounding_box()); print("PCB holes (housing coords):", holes(board))
o = import_step("orin_assm.step")
carrier = next(c for c in next(c for c in o.children if c.label.startswith("600-")).children if c.label.startswith("600-"))
print("carrier bbox", carrier.bounding_box()); print("carrier holes:", [x for x in holes(carrier) if x[0] > 2.4])
