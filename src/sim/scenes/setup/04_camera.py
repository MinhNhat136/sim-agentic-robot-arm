# Buoc 4 - Hai camera RGB-D.
#  - table_cam: camera co dinh doi dien robot, nhin xuong ban (giong RealSense D435, 640x480, fx = 615 px).
#  - wrist_cam: camera tren co tay, gan vao panda_hand (giong D405, 640x480, fx = 340 px), de chup them goc nhin.
# Anh RGB, depth, segmentation lay bang annotator luc chay; trong USD chi luu vi tri va noi tham so.

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

# ---- Table camera ------------------------------------------------------- #
stage = L.get_stage()
from pxr import UsdGeom

UsdGeom.Xform.Define(stage, "/World/cameras")
L.define_camera(stage, "/World/cameras/table_cam", eye=(1.35, 0.0, 0.80), target=(0.52, 0.0, 0.0),
                width=640, height=480, fx=615.0, near=0.05, far=10.0)

# ---- Wrist camera ------------------------------------------------------- #
# Trong he panda_hand: +Z la huong tien vao cua tay kep, dau ngon o z = 0.1034 m.
# Camera dat lech 7 cm theo +X, nhin cheo ve diem phia truoc dau ngon (thay duoc dau ngon o mep duoi anh).
L.define_camera(stage, "/World/franka/panda_hand/wrist_cam", eye=(0.07, 0.0, 0.04), target=(0.0, 0.0, 0.30),
                up=(1.0, 0.0, 0.0), width=640, height=480, fx=340.0, near=0.03, far=5.0)
print("Buoc 4 xong: /World/cameras/table_cam, /World/franka/panda_hand/wrist_cam")
