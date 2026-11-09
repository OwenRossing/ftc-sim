"""Field CAD cache (step_mesh.py) -> sim collision shapes.

usage: field_colliders.py field_mesh.npz out.json
Every shape is an oriented box in SI meters in the physics frame: x,y = Pedro field axes (0-3.6576),
z up. Solid field parts get one box. The Hive skins are folded sheets, so each flat facet gets its
own thin plate, which keeps the baskets hollow so balls can land inside them.
"""
import sys
import re
import json
import collections
import numpy as np

cache, out = sys.argv[1], sys.argv[2]
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
d = np.load(cache, allow_pickle=True)
H = 3.6576 / 2


def to_field(v):
    return np.column_stack([v[:, 0] + H, -v[:, 2] + H, v[:, 1]])


def obb(v, name):
    c0 = (v.min(axis=0) + v.max(axis=0)) / 2
    vals, vecs = np.linalg.eigh(np.cov((v - c0).T) + np.eye(3) * 1e-12)
    axes = vecs.T
    p = (v - c0) @ axes.T
    lo, hi = p.min(axis=0), p.max(axis=0)
    c = c0 + ((lo + hi) / 2) @ axes
    return {"name": name, "c": c.round(5).tolist(), "axes": axes.round(5).tolist(),
            "h": np.maximum((hi - lo) / 2, 0.002).round(5).tolist()}


def facets(v, f, name, min_area=0.002):
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    n = np.cross(b - a, c - a)
    area = np.linalg.norm(n, axis=1) / 2
    ok = area > 1e-9
    n[ok] /= (2 * area[ok])[:, None]
    groups = collections.defaultdict(list)
    for i in np.nonzero(ok)[0]:
        u = n[i] if n[i][np.argmax(np.abs(n[i]))] > 0 else -n[i]
        groups[tuple(np.round(u / 0.03).astype(int)) + (round(float(u @ a[i]) / 0.006),)].append(i)
    boxes, done = [], []
    for key, idx in sorted(groups.items(), key=lambda kv: -area[kv[1]].sum()):
        if area[idx].sum() < min_area:
            continue
        u = np.array(key[:3], dtype=float)
        u /= np.linalg.norm(u)
        # a sheet has two sides a few mm apart: only keep one plate per facet
        offset = key[3] * 0.006
        if any(abs(u @ w - 1) < 0.002 and abs(o - offset) < 0.012 for w, o in done):
            continue
        done.append((u, offset))
        boxes.append(obb(v[np.unique(f[idx])], name))
    return boxes


STATIC = re.compile(r"a-frame|frame foot|foot bar|acm panel|axle holder|flower (hips pipe|layer|field bracket|backstop|peanut)", re.I)
static, hives = [], {"red_hive": [], "blue_hive": []}
sizes = collections.Counter()
for name, v, f in zip(d["names"], d["verts"], d["faces"]):
    leaf = name.split("/")[-1]
    v = to_field(np.asarray(v, dtype=float))
    hive = "red_hive" if "Red Hive" in name else "blue_hive" if "Blue Hive" in name else None
    if hive:
        if re.search(r"Goal (Bottom|Top|Back) Skin", leaf):
            boxes = facets(v, np.asarray(f), leaf)
        elif re.search(r"Basket Base Tube", leaf):
            boxes = [obb(v, leaf)]
        # no "Goal Rib": the rims are rings around the opening, one solid box would fill the whole cell
        else:
            boxes = []
        hives[hive] += boxes
        sizes[hive] += len(boxes)
    elif STATIC.search(leaf):
        static.append(obb(v, leaf))
        sizes["static"] += 1

pieces = []
for p in json.load(open(cache + ".pieces.json")):
    lo, hi = np.array(p["lo"]), np.array(p["hi"])
    c = (lo + hi) / 2
    pieces.append({"type": "pollen" if "pollen" in p["name"] else "nectar",
                   "alliance": "red" if "red" in p["name"] else "blue" if "blue" in p["name"] else None,
                   "pos": [round(c[0] + H, 4), round(-c[2] + H, 4), round(c[1], 4)],
                   "radius": round(float((hi - lo).max()) / 2, 4)})
inside = [p for p in pieces if 0 < p["pos"][0] < 2 * H and 0 < p["pos"][1] < 2 * H]
json.dump({"static": static, "red_hive": hives["red_hive"], "blue_hive": hives["blue_hive"],
           "axle": {"z": 1.0931, "y": H}, "pieces": pieces}, open(out, "w"))
print("boxes", dict(sizes), "pieces", collections.Counter((p["type"], p["alliance"]) for p in pieces),
      "inside field", len(inside))
print("radii in", sorted({round(p["radius"] * 39.37, 2) for p in pieces}))
print("hive z range m", round(min(b["c"][2] for b in hives["red_hive"]), 2), round(max(b["c"][2] for b in hives["red_hive"]), 2))
for p in pieces[:6]:
    print(" ", p["type"], p["alliance"], [round(x * 39.37, 1) for x in p["pos"]])
