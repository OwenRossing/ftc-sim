"""Game piece GLBs -> light copies for the viewer (they're drawn up to 40 times).

usage: ball_export.py "folder with the Parts - am-585x GLBs" out_dir
Simplifies to about 700 triangles, centers it, and scales to the real radius in inches.
The viewer colors them itself, so only the shape matters.
"""
import sys
import numpy as np
import trimesh

src, out = sys.argv[1], sys.argv[2]
for file, name, radius in (("Parts - am-5851- Pollen.glb", "pollen", 1.4),
                           ("Parts - am-5852- Red Nectar.glb", "redNectar", 1.81),
                           ("Parts - am-5852- Blue Nectar.glb", "blueNectar", 1.81)):
    mesh = trimesh.load(f"{src}/{file}", force="mesh")  # these are Draco compressed, needs DracoPy
    # real size first: the simplifier struggles with tiny coordinates
    center = (mesh.vertices.max(axis=0) + mesh.vertices.min(axis=0)) / 2
    original_size = (mesh.vertices.max(axis=0) - mesh.vertices.min(axis=0)).max() / 2
    mesh.vertices = (mesh.vertices - center) * (radius / original_size)
    mesh.merge_vertices()
    mesh = mesh.simplify_quadric_decimation(face_count=700)
    center = (mesh.vertices.max(axis=0) + mesh.vertices.min(axis=0)) / 2
    size = (mesh.vertices.max(axis=0) - mesh.vertices.min(axis=0)).max() / 2
    mesh.vertices = (mesh.vertices - center) * (radius / size)
    mesh.fix_normals()
    mesh.export(f"{out}/{name}.glb")
    print(name, len(mesh.faces), "tris, source radius", round(float(original_size), 4))
