# shared: tools dir on sys.path, cwd = gitignored data dir cad/context/dofbot_fit
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "../../context/dofbot_fit"))
sys.path.insert(0, HERE)
os.makedirs(DATA, exist_ok=True)
os.chdir(DATA)
