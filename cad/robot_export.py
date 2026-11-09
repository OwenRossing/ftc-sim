"""Cached robot CAD (robot_mesh.py) -> one GLB per moving group, in the sim's robot frame.

usage: robot_export.py robot_tree.json robot_mesh.npz out_dir [--write]

CAD frame: Y up, +Z robot front, floor at y = 0.05 in (wheel bottoms), millimeters.
Robot frame (robot.json): inches, +x forward, +y up with the floor at 0, +z right.
    x_robot = z_cad, y_robot = y_cad + FLOOR_IN, z_robot = -x_cad

Without --write it only prints triangles per group.
"""
import sys
import re
import json
import collections
import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

tree_path, mesh_path, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
write = "--write" in sys.argv
sys.stdout.reconfigure(encoding="utf-8", errors="replace")

FLOOR_IN = -0.05  # the wheels bottom out at y = 0.05 in the CAD
MM_TO_IN = 1 / 25.4

# small hardware that would only cost triangles
DROP = re.compile(r"^\s*(Socket|Button|Countersunk) M\d|bearing|#10-32|m4 x|standoff|hex spacer|square beam|"
                  r"color sensor|encoder|ELC|origin cube", re.I)
# in the wheels only what you can see: the bearings and pins inside the rollers cost a lot
WHEEL_KEEP = re.compile(r"^Roller( <\d+>)?$|Wheel Core|Slant Plate|Belt Wheels|Wheel Hub", re.I)
FLYWHEEL = re.compile(r"stealth wheel|3D-Print Adapter|Rounded Hex|24t Custom HTD", re.I)
ROLLER = re.compile(r"ST Hub|60mm ST|432\.6mm", re.I)


def classify(row):
    path = "/".join(row["path"])
    name = row["name"]
    if len(row["path"]) == 1 and re.search(r"pollen|nectar", name, re.I):
        return None  # game pieces left in the assembly
    if "Wheel Assembly" in path:
        return "wheel" if WHEEL_KEEP.search(name) else None  # split by position below
    if DROP.search(name) or DROP.search(row["proto_name"]):
        return None
    if "Sorter/Shooter Assembly" in path:
        if "Energy Chain" in name:
            return "chassis"  # it flexes as the turret turns, close enough to leave it still
        if FLYWHEEL.search(name):
            return "flywheels"
        if "Pollen Hood" in name or "Hood Shaft Gear" in name:
            return "pollenHood"
        if "Nectar Hood" in name or name.startswith("Part 4"):
            return "nectarHood"
        return "turret"
    if "Flower Mech Assembly" in path:
        return "flower"
    if "Intake Assembly" in path:
        return "intakeRoller" if ROLLER.search(name) else "intake"
    if name.startswith("Gate v1"):
        return "gate"
    return "chassis"


table = json.load(open(tree_path, encoding="utf-8"))
data = np.load(mesh_path, allow_pickle=True)
proto_verts, proto_faces = data["verts"], data["faces"]

groups = collections.defaultdict(lambda: collections.defaultdict(lambda: ([], [])))
tris = collections.Counter()
for row in table:
    if row["proto"] < 0 or not row["lo"]:
        continue
    group = classify(row)
    if group is None:
        continue
    if group == "wheel":
        wheel_path = next(s for s in row["path"] if "Wheel Assembly" in s)
        group = wheel_path  # resolved to FL/FR/BL/BR once we know where each assembly sits
    m = np.array(row["matrix"])
    world = (proto_verts[row["proto"]] @ m[:3, :3].T + m[:3, 3]) * MM_TO_IN
    robot = np.column_stack([world[:, 2], world[:, 1] + FLOOR_IN, -world[:, 0]])
    faces = proto_faces[row["proto"]]
    color = tuple(row["color"] or (150, 155, 165))
    vs, fs = groups[group][color]
    offset = sum(len(v) for v in vs)
    vs.append(robot)
    fs.append(faces + offset)
    tris[group] += len(faces)

# wheel assemblies -> frontLeft etc, by where they actually are (robot left is -z in this frame)
names = {}
for group in list(groups):
    if "Wheel Assembly" not in group:
        continue
    pts = np.vstack([v for vs, _ in groups[group].values() for v in vs])
    c = (pts.min(axis=0) + pts.max(axis=0)) / 2
    side = "wheelF" if c[0] > 0 else "wheelB"
    side += "L" if c[2] < 0 else "R"
    names[group] = side
    print(f"{group} -> {side}, axle at x={c[0]:.2f} y={c[1]:.2f} z={c[2]:.2f}")
    groups[side] = groups.pop(group)
    tris[side] = tris.pop(group)

# how much detail to keep per group, the moving mechanisms stay sharp (needs fast-simplification)
KEEP_FRACTION = {"wheel": 0.08, "chassis": 0.25, "turret": 0.5}
DEFAULT_KEEP = 1.0
if "--low" in sys.argv:  # for slow laptops and phones, written to models/robot/low
    KEEP_FRACTION = {"wheel": 0.012, "chassis": 0.05, "turret": 0.12}
    DEFAULT_KEEP = 0.3
final_tris = collections.Counter()
info = {}
for group, colors in sorted(groups.items()):
    pts = np.vstack([v for vs, _ in colors.values() for v in vs])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    info[group] = {"lo": lo.round(2).tolist(), "hi": hi.round(2).tolist(), "tris": int(tris[group])}
    print(f"{group:13s} {tris[group]:8d} tris  lo {lo.round(1).tolist()}  hi {hi.round(1).tolist()}")
    if not write:
        continue
    scene = trimesh.Scene()
    for i, (color, (vs, fs)) in enumerate(colors.items()):
        mesh = trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=True)  # merges vertices for simplifying
        keep = KEEP_FRACTION.get(group[:5], KEEP_FRACTION.get(group, DEFAULT_KEEP))
        if keep < 1.0 and len(mesh.faces) > 500:
            mesh = mesh.simplify_quadric_decimation(face_count=max(300, int(len(mesh.faces) * keep)))
        final_tris[group] += len(mesh.faces)
        material = PBRMaterial(baseColorFactor=[*color, 255], metallicFactor=0.2, roughnessFactor=0.65, doubleSided=True)
        mesh.visual = trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(mesh, node_name=f"{group}_{i}", geom_name=f"{group}_{i}")
    scene.export(f"{out_dir}/{group}.glb")
print("total tris", sum(tris.values()), "after simplifying", sum(final_tris.values()))
