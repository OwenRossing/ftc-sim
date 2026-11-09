# CAD -> sim models

The sim viewer loads GLB. When CAD only exports STEP, convert it with OpenCascade (the same
geometry engine CAD viewers use). Run in a throwaway Python environment:

```
python -m venv cadenv
cadenv\Scripts\python -m pip install cadquery-ocp trimesh numpy
cadenv\Scripts\python step_mesh.py field.step field_mesh.npz
cadenv\Scripts\python field_export.py field_mesh.npz ..\web\models\field.glb ..\web\field_info.json
```

- `step_mesh.py` reads the STEP assembly (keeping full part names and colors), drops tiny
  hardware, floor tiles, screws, pins and the pre-placed game pieces, tessellates at ~12 mm and
  caches the triangles. The whole field takes ~15 s.
- `field_export.py` writes the GLB: static parts merged by color, the Red Hive and Blue Hive as
  their own nodes (`red_hive`, `blue_hive`) so they can tilt, plus `field_info.json` with the
  hive axle and the 4 goal AprilTag positions (meters, CAD frame: Y up, origin field center).
- gmsh was tried first and crashes on this field's CAD, don't bother.

## The robot

Export the assembly from Onshape as one STEP file (not per-part files, those lose the placement).

```
cadenv\Scripts\python -m pip install fast-simplification
cadenv\Scripts\python robot_mesh.py "Main Assembly.step" robot_tree.json robot_mesh.npz
cadenv\Scripts\python robot_export.py robot_tree.json robot_mesh.npz ..\web\models\robot --write
```

`robot_mesh.py` caches every part's path, placement and round-hole axes (that's how the hinges in
`sim/web/robot.json` were measured). `robot_export.py` sorts parts into moving groups by assembly
path (`classify()`) and writes them in the robot frame. If the CAD changes, re-run both; hinges in
robot.json only need updating if the mechanism moved.
