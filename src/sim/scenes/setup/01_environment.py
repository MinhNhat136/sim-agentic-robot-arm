import importlib
import os
import sys

import carb

carb.log_warn("00.Import standard lib") # ---- Standard library ------------------------------------ #

carb.log_warn("01.Import usd lib") # ---- Isaac Sim / USD ------------------------------------------ #
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

carb.log_warn("02.Get Scene Ref") # ---- Add scenes/ to sys.path ----------------------------------- # 

SCENES = os.path.dirname(omni.usd.get_context().get_stage().GetRootLayer().realPath)
assert SCENES, "Scene does not exist, create the empty scene first"
if SCENES not in sys.path:
    sys.path.insert(0, SCENES)

import scene_lib as L  # ---- Load scene_lib ------------------------------------------------------- #

importlib.reload(L)

carb.log_warn("03.Init common defs") # ---- Define Defs -------------------------------------------- #

def setup_stage(stage: Usd.Stage) -> Usd.Prim:
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    return world

def add_ground(stage: Usd.Stage, z: float, size: float = 20.0) -> None:
    path = "/World/ground" 
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    root = UsdGeom.Xform.Define(stage, path)
    L.set_pose(root.GetPrim(), (0.0, 0.0, z))
    s = size / 2.0

    mesh = UsdGeom.Mesh.Define(stage, path + "/mesh")
    mesh.CreatePointsAttr([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(0, 0, 1)] * 4)
    mesh.CreateDisplayColorAttr([(0.32, 0.33, 0.35)])
    plane = UsdPhysics.CollisionAPI.Apply(UsdGeom.Plane.Define(stage, path + "/collision").GetPrim())
    
    UsdGeom.Plane(plane.GetPrim()).CreateAxisAttr(UsdGeom.Tokens.z)
    UsdGeom.Plane(plane.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)

def add_lights(stage: Usd.Stage) -> None:
    for p in ("/World/lights/dome", "/World/lights/key"):
        if stage.GetPrimAtPath(p):
            stage.RemovePrim(p)
    UsdGeom.Xform.Define(stage, "/World/lights")
    dome = UsdLux.DomeLight.Define(stage, "/World/lights/dome")
    dome.CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, "/World/lights/key")
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(1.0)

    rot = Gf.Rotation(Gf.Vec3d(0, 1, 0), 35) * Gf.Rotation(Gf.Vec3d(0, 0, 1), -30)
    L.set_pose(key.GetPrim(), (0.0, 0.0, 3.0), quat=rot.GetQuat())

def add_physics_scene(stage: Usd.Stage, hz: int = 120) -> None:
    path = "/World/PhysicsScene"
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    scene = UsdPhysics.Scene.Define(stage, path)
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    prim = scene.GetPrim()
    prim.AddAppliedSchema("PhysxSceneAPI")
    prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(hz)

carb.log_warn("04.Build Env") # ---- Build Env------------------------------------------------------ #

stage = L.get_stage()
setup_stage(stage)  
# Stage moi tren GUI co san /Environment (den mac dinh cua template): xoa de anh sang do buoc nay quyet dinh.

if stage.GetPrimAtPath("/Environment"):
    stage.RemovePrim("/Environment")

carb.log_warn("05.Build table and robot stand")# ---- Build: table and robot stand ----------------- #
# He toa do: goc world = goc robot (panda_link0), +X ra truoc, +Y sang trai, mat ban o z = 0.
# Ban 0.70 x 1.00 m (x tu 0.20 den 0.90). Vi tri lay theo scene base_empty cua RoboLab.
L.add_reference(stage, "/World/env/table", L.robolab("fixtures/table_oak.usd"), xyz=(0.5471, 0.0, -0.35))
L.add_reference(stage, "/World/env/franka_table", L.robolab("fixtures/franka_table.usd"), xyz=(-0.087, 0.0, 0.0),
                yaw_deg=180.0)
L.add_label(stage.GetPrimAtPath("/World/env/table"), "table")


carb.log_warn("06.Build ground, lights, physics") # ---- Build: ground, lights, physics ----------- #
add_ground(stage, z=-0.697)
add_lights(stage)
add_physics_scene(stage, hz=120)  # buoc vat ly 120 Hz
