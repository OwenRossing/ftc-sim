"""Robot STEP -> parts table + cached triangles (read once, everything later works off the cache).

robot_tree.json: every leaf part: assembly path, world bbox (in), placement matrix, proto id,
                 cylinder axes (world, in) for finding hinges and shafts, triangle count.
robot_mesh.npz:  triangles per prototype in local coordinates (meters), so identical parts share.
Fastener-type hardware gets listed but not tessellated. Prints only a short summary.
"""
import sys
import re
import json
import time
import collections
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool, XCAFDoc_ColorType, XCAFDoc_ColorTool
from OCP.TDF import TDF_Label
from OCP.OCP.collections import Sequence_TDF_Label as Seq
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.Quantity import Quantity_Color
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.IFSelect import IFSelect_RetDone

step, out_json, out_npz = sys.argv[1], sys.argv[2], sys.argv[3]
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
t0 = time.time()
def log(msg):
    print(f"[{time.time() - t0:5.0f}s] {msg}", flush=True)

HARDWARE = re.compile(r"screw|locknut|nylock|\bnut\b|washer|e-clip|rivet|shim|roller pin|bolt", re.I)
MM_TO_IN = 1 / 25.4

doc = TDocStd_Document(TCollection_ExtendedString("doc"))
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
reader.SetColorMode(True)
if reader.ReadFile(step) != IFSelect_RetDone:
    sys.exit("couldn't read STEP")
reader.Transfer(doc)
log("read")
shapes = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def name_of(label):
    attr = TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ""


def color_of(label):
    c = Quantity_Color()
    for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if XCAFDoc_ColorTool.GetColor_s(label, kind, c):
            return [int(c.Red() * 255), int(c.Green() * 255), int(c.Blue() * 255)]
    return None


def label_key(label):
    tags = []
    while not label.IsRoot():
        tags.append(label.Tag())
        label = label.Father()
    return tuple(tags)


def matrix_of(loc):
    t = loc.Transformation()
    return [[t.Value(i, j) for j in range(1, 5)] for i in range(1, 4)]


leaves = []
def walk(label, loc, path, color):
    name = name_of(label)
    color = color_of(label) or color
    if XCAFDoc_ShapeTool.IsReference_s(label):
        ref = TDF_Label()
        XCAFDoc_ShapeTool.GetReferredShape_s(label, ref)
        walk(ref, loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(label)), path + [name], color)
    elif XCAFDoc_ShapeTool.IsAssembly_s(label):
        comps = Seq()
        XCAFDoc_ShapeTool.GetComponents_s(label, comps)
        for i in range(1, comps.Length() + 1):
            walk(comps.Value(i), loc, path, color)  # a reference names itself, skip the prototype's name
    else:
        leaves.append({"path": path, "name": path[-1] if path else name, "proto_name": name,
                       "key": label_key(label), "label": label, "matrix": matrix_of(loc), "color": color})


roots = Seq()
shapes.GetFreeShapes(roots)
for i in range(1, roots.Length() + 1):
    walk(roots.Value(i), TopLoc_Location(), [], None)
log(f"{len(leaves)} leaf parts, {len({str(l['key']) for l in leaves})} unique")

protos = {}
for leaf in leaves:
    key = str(leaf["key"])
    if key in protos:
        continue
    if HARDWARE.search(leaf["proto_name"]) or HARDWARE.search(leaf["name"]):
        protos[key] = None
        continue
    shape = XCAFDoc_ShapeTool.GetShape_s(leaf["label"])
    # mecanum rollers are 1.5M triangles at normal detail, nobody can see that
    coarse = "Wheel Assembly" in "/".join(leaf["path"])
    BRepMesh_IncrementalMesh(shape, 4.0 if coarse else 1.2, False, 1.2 if coarse else 0.9, True)
    verts, faces, axes, offset = [], [], {}, 0
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
            verts.append(pts)
            faces.append(idx + offset)
            offset += len(pts)
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() == GeomAbs_Cylinder:
            cyl = surf.Cylinder()
            if cyl.Radius() <= 40:
                o, d = cyl.Axis().Location(), cyl.Axis().Direction()
                k = (round(o.X(), 1), round(o.Y(), 1), round(o.Z(), 1), round(abs(d.X()), 2), round(abs(d.Y()), 2),
                     round(abs(d.Z()), 2), round(cyl.Radius(), 1))
                axes.setdefault(k, [o.X(), o.Y(), o.Z(), d.X(), d.Y(), d.Z(), cyl.Radius()])
        exp.Next()
    protos[key] = {"verts": np.vstack(verts) if verts else np.empty((0, 3)),
                   "faces": np.vstack(faces) if faces else np.empty((0, 3), dtype=np.int64),
                   "axes": list(axes.values())[:40]}
log(f"meshed {sum(1 for p in protos.values() if p)} protos, skipped {sum(1 for p in protos.values() if p is None)} hardware")

table = []
keys = sorted(k for k, p in protos.items() if p)
key_index = {k: i for i, k in enumerate(keys)}
for leaf in leaves:
    key = str(leaf["key"])
    proto = protos[key]
    row = {"path": leaf["path"], "name": leaf["name"], "proto_name": leaf["proto_name"], "color": leaf["color"],
           "matrix": leaf["matrix"], "proto": key_index.get(key, -1), "tris": 0, "lo": None, "hi": None, "axes": []}
    if proto is not None and len(proto["verts"]):
        m = np.array(leaf["matrix"])
        world = proto["verts"] @ m[:3, :3].T + m[:3, 3]
        row["lo"] = (world.min(axis=0) * MM_TO_IN).round(3).tolist()
        row["hi"] = (world.max(axis=0) * MM_TO_IN).round(3).tolist()
        row["tris"] = len(proto["faces"])
        for a in proto["axes"]:
            o = (m[:3, :3] @ np.array(a[:3]) + m[:3, 3]) * MM_TO_IN
            d = m[:3, :3] @ np.array(a[3:6])
            row["axes"].append([*o.round(3).tolist(), *d.round(3).tolist(), round(a[6] * MM_TO_IN, 3)])
    table.append(row)

json.dump(table, open(out_json, "w", encoding="utf-8"))
np.savez_compressed(out_npz, keys=np.array(keys, dtype=object),
                    verts=np.array([protos[k]["verts"].astype(np.float32) for k in keys] + [None], dtype=object)[:-1],
                    faces=np.array([protos[k]["faces"] for k in keys] + [None], dtype=object)[:-1])
log("saved")

placed = [r for r in table if r["lo"]]
lo = np.min([r["lo"] for r in placed], axis=0)
hi = np.max([r["hi"] for r in placed], axis=0)
print("overall (in) lo", lo.round(1).tolist(), "hi", hi.round(1).tolist(), "size", (hi - lo).round(1).tolist())
print("triangles if everything kept:", sum(r["tris"] for r in table))
tree = collections.Counter(tuple(r["path"][:2]) for r in table)
print("top of the tree (level 1 / level 2 : parts):")
for k, n in sorted(tree.items(), key=lambda kv: -kv[1])[:25]:
    print(f"  {n:4d}  " + " / ".join(s[:40] for s in k))
