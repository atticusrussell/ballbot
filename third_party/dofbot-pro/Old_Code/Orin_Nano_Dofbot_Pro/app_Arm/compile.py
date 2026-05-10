import py_compile
import os

py_compile.compile(file = "YahboomArm.py", cfile = "YahboomArm.pyc", optimize=-1)

os.system("chmod +x YahboomArm.pyc")
