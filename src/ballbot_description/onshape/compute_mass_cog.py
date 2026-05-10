#!/usr/bin/env python3
"""Compute total mass and centre of gravity from a URDF.

Walks the joint tree from a chosen root frame, transforms each link's
inertial CoM into that root frame, and reports the mass-weighted CoG.
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def _vec(s):
    return np.array([float(x) for x in s.split()]) if s else np.zeros(3)


def _rpy_to_R(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(rpy)
    T[:3, 3] = xyz
    return T


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("urdf")
    ap.add_argument("--root", default="base_link", help="frame for CoG report")
    args = ap.parse_args()

    root = ET.parse(args.urdf).getroot()

    children = {}
    for j in root.findall("joint"):
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        o = j.find("origin")
        xyz = _vec(o.get("xyz", "") if o is not None else "")
        rpy = _vec(o.get("rpy", "") if o is not None else "")
        children.setdefault(parent, []).append((child, _T(xyz, rpy)))

    inertials = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        o = inertial.find("origin")
        com = _vec(o.get("xyz", "") if o is not None else "")
        inertials[link.get("name")] = (mass, com)

    transforms = {args.root: np.eye(4)}
    stack = [args.root]
    while stack:
        cur = stack.pop()
        for child, T_pc in children.get(cur, []):
            transforms[child] = transforms[cur] @ T_pc
            stack.append(child)

    total = 0.0
    weighted = np.zeros(3)
    rows = []
    for link, (mass, com_local) in inertials.items():
        if link not in transforms:
            print(f"warn: {link} unreachable from {args.root}", file=sys.stderr)
            continue
        com_root = (transforms[link] @ np.append(com_local, 1.0))[:3]
        total += mass
        weighted += mass * com_root
        rows.append((link, mass, com_root))

    cog = weighted / total
    print(f"Root frame  : {args.root}")
    print(f"Total mass  : {total:.4f} kg")
    print(f"CoG (x,y,z) : ({cog[0]:+.4f}, {cog[1]:+.4f}, {cog[2]:+.4f}) m")
    print()
    print(f"{'link':<32s}{'mass [kg]':>11s}   {'x':>9s} {'y':>9s} {'z':>9s}")
    for link, mass, com in sorted(rows, key=lambda r: -r[1]):
        print(f"{link:<32s}{mass:>11.4f}   {com[0]:>+9.4f} {com[1]:>+9.4f} {com[2]:>+9.4f}")


if __name__ == "__main__":
    main()
