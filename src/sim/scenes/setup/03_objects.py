# Buoc 3 - Vat the tren ban (assets RoboLab: HOPE, YCB, khoi mau, thung xam).
# Chon de thu cac kieu lenh: vat chinh cho lat cat doc (chai mu tat), mau sac ("coc do", "khoi xanh"),
# trai/phai (hai khoi), tay cam (coc), dat vao thung. Cac vat cach nhau >= 15 cm, nam trong tam voi
# cua Franka (x 0.38 - 0.66 m) va vua do mo kep 8 cm theo it nhat mot chieu.

# ---- Imports ------------------------------------------------------------ #
import importlib
import os
import sys

import omni.usd

# ---- Add scenes/ to sys.path -------------------------------------------- #
SCENES = os.path.dirname(omni.usd.get_context().get_stage().GetRootLayer().realPath)
if SCENES not in sys.path:
    sys.path.insert(0, SCENES)

# ---- Load scene_lib ----------------------------------------------------- #
import scene_lib as L

importlib.reload(L)

# ---- Object list -------------------------------------------------------- #
stage = L.get_stage()
from pxr import Vt

# ten prim, file trong robolab/RoboLab/assets, (x, y, z) he robot, goc yaw (do), ten goi trong lenh.
# z la do cao do duoc khi vat nam yen sau 2 s mo phong (verify_scene.py), nen bam Play vat khong nhay.
OBJECTS = [
    ("mustard_bottle", "objects/hope/mustard_bottle.usd", (0.52, 0.02, 0.0823), 20.0, ["mustard bottle"]),
    ("red_mug", "objects/ycb/mug.usd", (0.42, 0.22, 0.0435), 90.0, ["red mug", "red cup"]),
    ("red_block", "objects/basic/red_block.usd", (0.66, -0.06, 0.0263), 0.0, ["red block", "red cube"]),
    ("blue_block", "objects/basic/blue_block.usd", (0.66, 0.16, 0.0263), 0.0, ["blue block", "blue cube"]),
    ("tomato_soup_can", "objects/ycb/tomato_soup_can.usd", (0.38, -0.10, 0.0539), 0.0, ["tomato soup can"]),
    ("grey_bin", "fixtures/grey_bin.usd", (0.55, -0.34, 0.003), 0.0, ["grey bin", "box"]),
]

# ---- Add objects to stage ----------------------------------------------- #
for seg_id, (name, usd, xyz, yaw, aliases) in enumerate(OBJECTS, start=101):
    prim = L.add_reference(stage, f"/World/objects/{name}", L.robolab(usd), xyz=xyz, yaw_deg=yaw)
    L.add_label(prim, name)
    prim.SetCustomDataByKey("shimizu:seg_id", seg_id)  # id trong seg.png, giong fixture GraspGenX (vat tu 101)
    prim.SetCustomDataByKey("shimizu:aliases", Vt.StringArray(aliases))
    prim.SetCustomDataByKey("shimizu:source", "robolab/" + usd)
print("Buoc 3 xong:", len(OBJECTS), "vat:", ", ".join(o[0] for o in OBJECTS))
