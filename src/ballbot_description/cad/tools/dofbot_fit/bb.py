import _env  # noqa: F401  (cwd -> cad/context/dofbot_fit)
import sys, json
sys.path.insert(0, "/home/atticus/workspace/ballbot/src/ballbot_description/cad/tools")
from onshape_step_sync import Onshape, DocRef
D = "833f9762904995f22ff7dd9f"
api = Onshape("https://cad.onshape.com")
