"""goBILDA StarterBot CAD (robot_mesh.py cache) -> one GLB per moving part, in the sim's robot frame.

usage: starterbot_export.py robot_tree.json robot_mesh.npz out_dir [--low]

Its STEP is 3200-2627-0003.step from goBILDA. CAD frame: inches after robot_mesh.py, X = robot left,
-Y = forward, Z up with the floor at z = -1.89 (the bottom of the 96 mm wheels), middle wheels at y = 0.
Robot frame (web/robots/*.json): +x forward, +y up with the floor at 0, +z right.
    x_robot = -y_cad, y_robot = z_cad + 1.89, z_robot = -x_cad
Parts are named by goBILDA part number, so the moving ones are picked by number and where they sit.
"""
import sys
import re
import json
import collections
import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

tree_path, mesh_path, out_dir = sys.argv[1], sys.argv[2], sys.argv[3]
low = "--low" in sys.argv
sys.stdout.reconfigure(encoding="utf-8", errors="replace")

FLOOR = 1.89
MM_TO_IN = 1 / 25.4


def to_robot(p):
    return np.column_stack([-p[:, 1], p[:, 2] + FLOOR, -p[:, 0]])


def classify(row, center):
    """center: the part's center in the robot frame"""
    top = row["path"][0] if row["path"] else row["name"]
    x, y, z = center
    if top.startswith("Hardware") or top.startswith("Zip-Ties"):
        return None  # 280 screws and nuts, all triangles and no looks
    if top.startswith("3626-0014-0096") or top.startswith("3624-0014-0096"):  # 96 mm wheels and omnis
        if y > 5:
            return "launcherWheel"  # the one up top
        side = "L" if z < 0 else "R"
        pos = "F" if x > 3 else "B" if x < -3 else "M"
        return f"wheel{pos}{side}"
    if re.match(r"3613-4008-0048|2106-4008-2400|1522-0010-0120|2920-0001-4008:[34]|2303-4008-0020", top):
        return "gecko"  # the 48 mm Gecko wheels on their shaft, with its spacers and collars
    if top.startswith("3613-0014-0072") or top.startswith("3217-0001-2501 w.Hardware (2.6):1") \
            or top.startswith("3217-0001-2501 w.Hardware (2.6):2"):
        return "cornerL" if z < 0 else "cornerR"  # the 72 mm corner wheels on their servo hubs
    if top.startswith("1310-0016-4008:5"):
        return "launcherWheel"  # its hub
    return "chassis"


table = json.load(open(tree_path, encoding="utf-8"))
data = np.load(mesh_path, allow_pickle=True)
proto_verts, proto_faces = data["verts"], data["faces"]
# per triangle face colors, -1 where a face has none (older caches don't have them)
proto_colors = data["colors"] if "colors" in data.files else None

groups = collections.defaultdict(lambda: collections.defaultdict(lambda: ([], [])))
for row in table:
    if row["proto"] < 0 or not row["lo"]:
        continue
    m = np.array(row["matrix"])
    world = (proto_verts[row["proto"]] @ m[:3, :3].T + m[:3, 3]) * MM_TO_IN
    robot = to_robot(world)
    center = (robot.min(axis=0) + robot.max(axis=0)) / 2
    group = classify(row, center)
    if group is None:
        continue
    part_color = tuple(row["color"] or (150, 155, 165))
    faces = proto_faces[row["proto"]]
    tri_colors = proto_colors[row["proto"]] if proto_colors is not None else None
    if tri_colors is None or len(tri_colors) != len(faces):
        tri_colors = np.full((len(faces), 3), -1, dtype=np.int16)
    # a face's own color wins over the part's (black tread on a grey hub, a label on a motor)
    keys = [tuple(c) if c[0] >= 0 else part_color for c in tri_colors.tolist()]
    by_color = collections.defaultdict(list)
    for i, k in enumerate(keys):
        by_color[k].append(i)
    for color, idx in by_color.items():
        vs, fs = groups[group][color]
        offset = sum(len(v) for v in vs)
        sub = faces[idx]
        used = np.unique(sub)  # only the vertices these triangles use
        remap = np.full(len(robot), -1, dtype=np.int64)
        remap[used] = np.arange(len(used))
        vs.append(robot[used])
        fs.append(remap[sub] + offset)

# triangles to keep per group: the chassis is where the detail is, a wheel is round from far away
BUDGET = {"chassis": 90000, "wheel": 2500, "gecko": 6000, "corner": 1500, "launcherWheel": 3000}
scale = 0.2 if low else 1.0
info = {}
total = 0
for group, colors in sorted(groups.items()):
    pts = np.vstack([v for vs, _ in colors.values() for v in vs])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    tris = sum(len(f) for _, fs in colors.values() for f in fs)
    budget = int(BUDGET.get(group, BUDGET.get(group.rstrip("LR")[:6] if group.startswith("corner") else group[:5], 5000)) * scale)
    scene = trimesh.Scene()
    kept = 0
    for i, (color, (vs, fs)) in enumerate(sorted(colors.items(), key=lambda kv: -sum(len(f) for f in kv[1][1]))):
        mesh = trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=True)
        share = max(200, int(budget * len(mesh.faces) / tris))
        if len(mesh.faces) > share:
            mesh = mesh.simplify_quadric_decimation(face_count=share)
        kept += len(mesh.faces)
        material = PBRMaterial(baseColorFactor=[*color, 255], metallicFactor=0.25, roughnessFactor=0.6, doubleSided=True)
        mesh.visual = trimesh.visual.TextureVisuals(material=material)
        scene.add_geometry(mesh, node_name=f"{group}_{i}", geom_name=f"{group}_{i}")
    scene.export(f"{out_dir}/{group}.glb")
    total += kept
    c = (lo + hi) / 2
    info[group] = {"center": c.round(2).tolist(), "lo": lo.round(2).tolist(), "hi": hi.round(2).tolist()}
    print(f"{group:14s} {tris:8d} -> {kept:6d} tris  center {c.round(2).tolist()}  size {(hi - lo).round(2).tolist()}")
print("total", total)
json.dump(info, open(f"{out_dir}/parts.json", "w"), indent=1)
