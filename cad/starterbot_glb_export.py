"""goBILDA's mecanum StarterBot GLB (3200-2627-0004) -> one GLB per moving part, in the sim's robot frame.

usage: starterbot_glb_export.py Starter.glb out_dir [--low]

The GLB is meters, Y up, the robot facing +Z with its left at +X, the floor at the bottom of the mecanum wheels.
Robot frame (web/robots/*.json): inches, +x forward, +y up with the floor at 0, +z right, origin between the wheels.
Parts are picked by where they sit (the moving ones are wheels, the Gecko intake, the corner wheels, the launcher
wheel); screws are dropped and every group is simplified to a triangle budget so a laptop can draw it.
"""
import sys
import re
import json
import collections
import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

src, out_dir = sys.argv[1], sys.argv[2]
low = "--low" in sys.argv
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
IN = 39.3701

DROP = re.compile(r"screw|phillips|99461A|nut\b|washer|M3 x|M4 x|zip", re.I)

scene = trimesh.load(src, force="scene")
parts = []  # (name, verts in GLB inches, faces, color)
for node in scene.graph.nodes_geometry:
    T, g = scene.graph[node]
    mesh = scene.geometry[g]
    if DROP.search(node) or DROP.search(g):
        continue
    v = trimesh.transform_points(mesh.vertices, T) * IN
    try:
        color = tuple(int(c) for c in mesh.visual.material.baseColorFactor[:3])
    except Exception:
        color = (150, 155, 165)
    parts.append((node, v, np.asarray(mesh.faces), color))
print(len(parts), "parts kept")

# GLB inches -> robot frame: forward = +z, up = +y, right = -x
floor = -min(p[1][:, 1].min() for p in parts)


def to_robot(v):
    return np.column_stack([v[:, 2], v[:, 1] + floor, -v[:, 0]])


robot_parts = [(n, to_robot(v), f, c) for n, v, f, c in parts]

# wheel centers from the mecanum roller tires, then the origin halfway between the wheels
tires = [p for p in robot_parts if p[0].startswith("RollerTire")]
centers = collections.defaultdict(list)
for n, v, f, c in tires:
    m = v.mean(axis=0)
    centers[("F" if m[0] > np.median([t[1].mean(axis=0)[0] for t in tires]) else "B",
             "L" if m[2] < 0 else "R")].append(v)
wheel_center = {}
for (fb, lr), vs in centers.items():
    pts = np.vstack(vs)
    wheel_center[f"wheel{fb}{lr}"] = (pts.min(axis=0) + pts.max(axis=0)) / 2
mid = np.mean(list(wheel_center.values()), axis=0)
shift = np.array([mid[0], 0, mid[2]])
robot_parts = [(n, v - shift, f, c) for n, v, f, c in robot_parts]
wheel_center = {k: c - shift for k, c in wheel_center.items()}
print("wheels:", {k: c.round(2).tolist() for k, c in wheel_center.items()})


def center_of(names):
    pts = np.vstack([v for n, v, f, c in robot_parts if n.startswith(names)])
    return (pts.min(axis=0) + pts.max(axis=0)) / 2


gecko = center_of("3613-4008-0048")
launcher = center_of("3626-0014-0096")
corner_pts = [v for n, v, f, c in robot_parts if n.startswith("3613-0014-0072")]
corners = {}
for v in corner_pts:
    c = (v.min(axis=0) + v.max(axis=0)) / 2
    corners["cornerL" if c[2] < 0 else "cornerR"] = c
print("gecko", gecko.round(2).tolist(), "launcher", launcher.round(2).tolist(),
      {k: c.round(2).tolist() for k, c in corners.items()})


def classify(lo, hi):
    c = (lo + hi) / 2
    for name, w in wheel_center.items():  # anything inside a wheel's drum spins with it
        if abs(c[0] - w[0]) < 2.2 and abs(c[1] - w[1]) < 2.2 and abs(c[2] - w[2]) < 1.2 and hi[1] < w[1] + 2.3:
            return name
    if abs(c[0] - gecko[0]) < 1.0 and abs(c[1] - gecko[1]) < 1.0 and hi[1] - lo[1] < 2.1:
        return "gecko"  # the Gecko wheels, their shaft and spacers
    for name, k in corners.items():
        if abs(c[0] - k[0]) < 1.6 and abs(c[2] - k[2]) < 1.6 and abs(c[1] - k[1]) < 0.8:
            return name
    if abs(c[0] - launcher[0]) < 2.0 and abs(c[1] - launcher[1]) < 2.0 and abs(c[2] - launcher[2]) < 0.8:
        return "launcherWheel"
    return "chassis"


groups = collections.defaultdict(lambda: collections.defaultdict(lambda: ([], [])))
for n, v, f, c in robot_parts:
    g = classify(v.min(axis=0), v.max(axis=0))
    vs, fs = groups[g][c]
    off = sum(len(x) for x in vs)
    vs.append(v)
    fs.append(f + off)

BUDGET = {"chassis": 140000, "wheel": 9000, "gecko": 10000, "corner": 2500, "launcherWheel": 4000}
scale = 0.25 if low else 1.0
info = {"wheels": {}, "pivots": {}}
total = 0
for g, colors in sorted(groups.items()):
    tris = sum(len(x) for _, fs in colors.values() for x in fs)
    budget = int(BUDGET.get(g, BUDGET.get(g[:5], BUDGET.get(g[:6], 5000))) * scale)
    out = trimesh.Scene()
    kept = 0
    for i, (color, (vs, fs)) in enumerate(sorted(colors.items(), key=lambda kv: -sum(len(x) for x in kv[1][1]))):
        mesh = trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=True)
        share = int(budget * len(mesh.faces) / tris)
        if share < 60:
            continue  # a sliver of a color that would only cost a draw call
        if len(mesh.faces) > share:
            mesh = mesh.simplify_quadric_decimation(face_count=share)
        kept += len(mesh.faces)
        mesh.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
            baseColorFactor=[*color, 255], metallicFactor=0.3, roughnessFactor=0.55, doubleSided=True))
        out.add_geometry(mesh, node_name=f"{g}_{i}", geom_name=f"{g}_{i}")
    out.export(f"{out_dir}/{g}.glb")
    total += kept
    print(f"{g:14s} {tris:9d} -> {kept:6d} tris")
pivots = {**wheel_center, "gecko": gecko, "launcherWheel": launcher, **corners}
info = {k: c.round(2).tolist() for k, c in pivots.items()}
json.dump(info, open(f"{out_dir}/pivots.json", "w"), indent=1)
print("total", total, "pivots", info)
