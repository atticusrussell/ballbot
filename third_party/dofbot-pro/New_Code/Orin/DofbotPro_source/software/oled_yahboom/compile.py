#!/usr/bin/env python3
# coding=utf-8
"""
# file=input name cfile=output name
# python3 compile.py
"""

import py_compile
import os
py_compile.compile(file="yahboom_oled.py", cfile="yahboom_oled.pyc", optimize=-1)

os.system("chmod +x yahboom_oled.pyc")
