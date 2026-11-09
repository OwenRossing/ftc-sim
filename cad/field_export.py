"""Stage 2: cached triangles -> field.glb + field_tags.json (fast, no gmsh).

usage: field_export.py cache.npz out.glb out_info.json
Static parts are merged by color (few draw calls). Everything under the Red Hive / Blue Hive
assemblies (the goals that tip on the A-frame axle) becomes its own node so the viewer can tilt
it. The axle comes from the two Axle Holder parts. Colors are materials, so no scipy needed.
"""
import sys
import re
import json
import collections
import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

cache, out_glb, out_tags = sys.argv[1], sys.argv[2], sys.argv[3]
data = np.load(cache, allow_pickle=True)
names, colors, verts, faces = data["names"], data["colors"], data["verts"], data["faces"]

def color_for(name, rgba):
    leaf = name.rsplit("/", 1)[-1].lower()
    # clear stuff first: the CAD gives these an opaque color. The Hive skins are the polycarb
    if "glass" in leaf or "polycarb" in leaf or " skin" in leaf:
        return (200, 225, 240, 70)
    r, g, b, a = (int(c) for c in rgba)
    if (r, g, b) != (0, 0, 0):
        return (r, g, b, a or 255)
    n = name.lower()
    if "april tag" in n:
        return (235, 235, 235, 255)
    if "red" in n:
        return (200, 50, 50, 255)
    if "blue" in n:
        return (50, 90, 200, 255)
    if "nectar" in n:
        return (230, 120, 40, 255)
    if "pollen" in n:
        return (250, 210, 60, 255)
    if "gaffer" in n:
        return (30, 30, 30, 255)
    return (165, 170, 178, 255)

groups = collections.defaultdict(lambda: ([], []))
tags = []
axle_holders = []
seesaw_tris = 0
for name, rgba, v, f in zip(names, colors, verts, faces):
    color = color_for(name, rgba)
    kind = "red_hive" if "Red Hive" in name else "blue_hive" if "Blue Hive" in name else "static"
    key = (kind, color)
    vs, fs = groups[key]
    offset = sum(len(x) for x in vs)
    vs.append(v)
    fs.append(f + offset)
    if kind != "static":
        seesaw_tris += len(f)
    if "Axle Holder" in name:
        lo, hi = v.min(axis=0), v.max(axis=0)
        axle_holders.append((lo + hi) / 2)
    if "april tag" in name.lower():
        lo, hi = v.min(axis=0), v.max(axis=0)
        tags.append({"name": name.split("/")[-1], "center_m": ((lo + hi) / 2).round(4).tolist(),
                     "size_m": (hi - lo).round(4).tolist()})

scene = trimesh.Scene()
# one empty parent node per hive, every colored piece of that hive hangs under it
for hive in ("red_hive", "blue_hive"):
    scene.graph.update(frame_from=scene.graph.base_frame, frame_to=hive, matrix=np.eye(4))
counts = collections.Counter()
for i, ((kind, color), (vs, fs)) in enumerate(groups.items()):
    # field_export.py ... --keep 0.15 makes the low detail version (needs fast-simplification)
    keep = float(sys.argv[sys.argv.index("--keep") + 1]) if "--keep" in sys.argv else 1.0
    mesh = trimesh.Trimesh(np.vstack(vs).astype(np.float64), np.vstack(fs), process=keep < 1)
    if keep < 1 and len(mesh.faces) > 500:
        mesh = mesh.simplify_quadric_decimation(face_count=max(200, int(len(mesh.faces) * keep)))
    material = PBRMaterial(baseColorFactor=list(color), metallicFactor=0.1, roughnessFactor=0.75,
                           alphaMode="BLEND" if color[3] < 255 else "OPAQUE", doubleSided=True)
    mesh.visual = trimesh.visual.TextureVisuals(material=material)
    node = f"{kind}_part_{i}"
    parent = kind if kind != "static" else None
    scene.add_geometry(mesh, node_name=node, geom_name=node, parent_node_name=parent)
    counts[kind] += len(mesh.faces)
scene.export(out_glb)
axle = None
if len(axle_holders) >= 2:
    a, b = axle_holders[0], axle_holders[1]
    d = (b - a) / np.linalg.norm(b - a)
    axle = {"point_m": ((a + b) / 2).round(4).tolist(), "direction": d.round(4).tolist()}
print("hive axle:", axle)
json.dump({"units": "meters, field CAD frame (Y up, origin field center)", "hiveAxle": axle, "tags": tags},
          open(out_tags, "w"), indent=2)
print(f"triangles {dict(counts)}, seesaw parts tris={seesaw_tris}, tags={len(tags)}")
for t in tags:
    print("  tag", t["name"], t["center_m"], t["size_m"])
