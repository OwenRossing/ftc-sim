"""A robot_mesh.py cache -> one GLB of the whole robot, for viewing it anywhere (Blender, Windows 3D Viewer, a site).

usage: step_to_glb.py robot_tree.json robot_mesh.npz out.glb [--max-tris N] [--z-up]

Every part is its own node, named by its CAD name, in its CAD color (a face's own color wins). glTF is meters with
Y up, so the CAD's millimeters become meters and a Z-up CAD gets turned Y-up (--z-up, the goBILDA StarterBot is).
Big parts get simplified so the whole robot fits in --max-tris (default 600k); small ones are kept as they are.
Screws and nuts were already left out by robot_mesh.py.
"""
import sys
import json
import collections
import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

tree_path, mesh_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
max_tris = int(sys.argv[sys.argv.index("--max-tris") + 1]) if "--max-tris" in sys.argv else 600000
z_up = "--z-up" in sys.argv
sys.stdout.reconfigure(encoding="utf-8", errors="replace")

table = json.load(open(tree_path, encoding="utf-8"))
data = np.load(mesh_path, allow_pickle=True)
proto_verts, proto_faces = data["verts"], data["faces"]
proto_colors = data["colors"] if "colors" in data.files else None

placed = [r for r in table if r["proto"] >= 0 and r["lo"]]
total = sum(r["tris"] for r in placed)
keep = min(1.0, max_tris / total)
print(f"{len(placed)} parts, {total} triangles, keeping about {keep:.0%}")

materials = {}


def material(color):
    if color not in materials:
        materials[color] = PBRMaterial(baseColorFactor=[*color, 255], metallicFactor=0.3, roughnessFactor=0.55,
                                       doubleSided=True)
    return materials[color]


scene = trimesh.Scene()
names = collections.Counter()
kept = 0
for row in placed:
    m = np.array(row["matrix"])
    pts = (proto_verts[row["proto"]] @ m[:3, :3].T + m[:3, 3]) / 1000.0  # mm -> m
    if z_up:
        pts = np.column_stack([pts[:, 0], pts[:, 2], -pts[:, 1]])
    faces = proto_faces[row["proto"]]
    part_color = tuple(row["color"] or (150, 155, 165))
    tri_colors = proto_colors[row["proto"]] if proto_colors is not None else None
    if tri_colors is None or len(tri_colors) != len(faces):
        tri_colors = np.full((len(faces), 3), -1, dtype=np.int16)
    by_color = collections.defaultdict(list)
    for i, c in enumerate(tri_colors.tolist()):
        by_color[tuple(c) if c[0] >= 0 else part_color].append(i)

    base = row["name"] or row["proto_name"] or "part"
    names[base] += 1
    node = base if names[base] == 1 else f"{base} ({names[base]})"
    for j, (color, idx) in enumerate(by_color.items()):
        sub = faces[idx]
        used = np.unique(sub)
        remap = np.full(len(pts), -1, dtype=np.int64)
        remap[used] = np.arange(len(used))
        mesh = trimesh.Trimesh(pts[used], remap[sub], process=True)
        target = int(len(mesh.faces) * keep)
        if keep < 1.0 and len(mesh.faces) > 2000 and target >= 300:
            mesh = mesh.simplify_quadric_decimation(face_count=target)
        mesh.visual = trimesh.visual.TextureVisuals(material=material(color))
        name = node if len(by_color) == 1 else f"{node} [{j}]"
        scene.add_geometry(mesh, node_name=name, geom_name=name)
        kept += len(mesh.faces)

scene.export(out_path)
print(f"wrote {out_path}: {kept} triangles, {len(materials)} colors")
