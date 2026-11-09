"""Stage 1 (OpenCascade version): STEP -> per-part triangles cached in an .npz.

Same output as field_mesh.py (names, colors 0-255, verts in meters, faces) but uses OCC's own
BRepMesh tessellator, which is what CAD viewers use and doesn't crash on bad surfaces.
Walks the XCAF assembly tree so names keep their full path (Red Hive/..., Blue Hive/...).
"""
import sys
import time
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool, XCAFDoc_ColorType, XCAFDoc_ColorTool
from OCP.TDF import TDF_Label
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.Quantity import Quantity_Color
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.IFSelect import IFSelect_RetDone

step, out = sys.argv[1], sys.argv[2]
t0 = time.time()
def log(msg):
    print(f"[{time.time() - t0:6.0f}s] {msg}", flush=True)

doc = TDocStd_Document(TCollection_ExtendedString("doc"))
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
reader.SetColorMode(True)
if reader.ReadFile(step) != IFSelect_RetDone:
    sys.exit("couldn't read STEP")
reader.Transfer(doc)
log("read")

shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
colors = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

def name_of(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""

def color_of(label):
    c = Quantity_Color()
    for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if XCAFDoc_ColorTool.GetColor_s(label, kind, c):
            return (int(c.Red() * 255), int(c.Green() * 255), int(c.Blue() * 255), 255)
    return None

def corners(box):
    a, b = box.CornerMin(), box.CornerMax()
    return a.X(), a.Y(), a.Z(), b.X(), b.Y(), b.Z()

leaves = []  # (path, shape located in the world, color)
def walk(label, loc, path, color):
    name = name_of(label)
    color = color_of(label) or color
    if XCAFDoc_ShapeTool.IsReference_s(label):
        ref = TDF_Label()
        XCAFDoc_ShapeTool.GetReferredShape_s(label, ref)
        walk(ref, loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(label)), path + [name], color)
    elif XCAFDoc_ShapeTool.IsAssembly_s(label):
        comps = TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(label, comps)
        for i in range(1, comps.Length() + 1):
            walk(comps.Value(i), loc, path + [name], color)
    else:
        shape = XCAFDoc_ShapeTool.GetShape_s(label)
        leaves.append(("/".join(p for p in path + [name] if p), shape.Moved(loc), color))

roots = TDF_LabelSequence()
shapes.GetFreeShapes(roots)
for i in range(1, roots.Length() + 1):
    walk(roots.Value(i), TopLoc_Location(), [], None)
log(f"{len(leaves)} parts")

# OCC works in millimeters here; figure out the scale from the overall size (field ~3.66 m)
total = Bnd_Box()
for _, shape, _ in leaves:
    BRepBndLib.Add_s(shape, total)
x0, y0, z0, x1, y1, z1 = corners(total)
to_m = 0.001 if max(x1 - x0, z1 - z0) > 100 else 1.0
log(f"extent {x1 - x0:.1f} x {y1 - y0:.1f} x {z1 - z0:.1f} (scale to meters {to_m})")

names_out, colors_out, verts_out, faces_out = [], [], [], []
pieces = []
tri_total = 0
skipped = 0
for path, shape, color in leaves:
    low = path.lower()
    leaf = low.rsplit("/", 1)[-1]  # only judge a part by its own name: its assembly can be "Riveted FTC Panel"
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    if box.IsVoid():
        continue
    bx0, by0, bz0, bx1, by1, bz1 = corners(box)
    diag = ((bx1 - bx0) ** 2 + (by1 - by0) ** 2 + (bz1 - bz0) ** 2) ** 0.5 * to_m
    # remember where the game pieces start, they aren't meshed (the sim draws its own)
    if leaf.endswith(": pollen") or "nectar" in leaf:
        pieces.append({"name": leaf, "lo": [bx0 * to_m, by0 * to_m, bz0 * to_m], "hi": [bx1 * to_m, by1 * to_m, bz1 * to_m]})
    is_rivet = "rivet" in leaf and "rail" not in leaf and "panel" not in leaf
    if (diag < 0.02 or "soft tiles" in leaf or is_rivet or "sticker" in leaf or "screw" in leaf
            or leaf.endswith(": pollen") or "nectar" in leaf or "quick release pin" in leaf):
        skipped += 1
        continue
    BRepMesh_IncrementalMesh(shape, 12.0 / to_m * 0.001, False, 0.9, True)  # ~12 mm, 0.9 rad: plenty for a viewer
    verts, faces, offset = [], [], 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            pts = np.empty((tri.NbNodes(), 3))
            for n in range(1, tri.NbNodes() + 1):
                p = tri.Node(n).Transformed(trsf)
                pts[n - 1] = (p.X(), p.Y(), p.Z())
            idx = np.empty((tri.NbTriangles(), 3), dtype=np.int64)
            for t in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(t).Get()
                idx[t - 1] = (a - 1, b - 1, c - 1)
            if face.Orientation() == TopAbs_REVERSED:
                idx = idx[:, ::-1]
            verts.append(pts * to_m)
            faces.append(idx + offset)
            offset += len(pts)
        exp.Next()
    if not faces:
        continue
    names_out.append(path)
    colors_out.append(color or (0, 0, 0, 0))
    verts_out.append(np.vstack(verts).astype(np.float32))
    faces_out.append(np.vstack(faces))
    tri_total += len(faces_out[-1])
log(f"meshed {len(names_out)} parts ({skipped} skipped), {tri_total} triangles")

np.savez_compressed(out, names=np.array(names_out, dtype=object), colors=np.array(colors_out),
                    verts=np.array(verts_out, dtype=object), faces=np.array(faces_out, dtype=object))
import json
json.dump(pieces, open(str(out) + ".pieces.json", "w"))
log(f"cached {out}, {len(pieces)} game pieces")
