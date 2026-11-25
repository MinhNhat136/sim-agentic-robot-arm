# Buoc 2 - Robot Franka Panda (tay kep Panda goc, giong gripper "franka_panda" cua GraspGenX)
# dat o goc world, tren gia ke, tu the "ready".

# ---- Imports ------------------------------------------------------------ #
import importlib
import os
import sys

import carb
import omni.usd

# ---- Add scenes/ to sys.path -------------------------------------------- #
SCENES = os.path.dirname(omni.usd.get_context().get_stage().GetRootLayer().realPath)
if SCENES not in sys.path:
    sys.path.insert(0, SCENES)

# ---- Load scene_lib ----------------------------------------------------- #
import scene_lib as L

importlib.reload(L)

# ---- Add Franka robot --------------------------------------------------- #
stage = L.get_stage()
ROBOT = "/World/franka"
L.add_reference(stage, ROBOT, L.franka_url(), xyz=(0.0, 0.0, 0.0))  # franka.usd cua Isaac Sim 6.0 (can Internet)
L.add_label(stage.GetPrimAtPath(ROBOT), "robot")

# ---- Ready pose (arm joints) -------------------------------------------- #
# Tu the ready (do). Asset goc de moi khop o 0 do, ma khop 4 phai nam trong [-176, -4] do,
# nen phai dat lai ca vi tri ban dau lan dich cua drive, neu khong bam Play robot se giat.
READY_DEG = [0.0, -45.0, 0.0, -135.0, 0.0, 90.0, 45.0]
for i, q in enumerate(READY_DEG, start=1):
    L.set_joint(stage, f"{ROBOT}/panda_link{i - 1}/panda_joint{i}", q)

# ---- Open gripper ------------------------------------------------------- #
# Kep mo het: moi ngon 0.04 m. Ngon 2 la mimic joint (di theo ngon 1) nen chi dat vi tri ban dau.
L.set_joint(stage, f"{ROBOT}/panda_hand/panda_finger_joint1", 0.04, angular=False)
L.set_joint(stage, f"{ROBOT}/panda_hand/panda_finger_joint2", 0.04, angular=False, drive=False)
print("Buoc 2 xong: Franka o", ROBOT, "tu the ready", READY_DEG)
