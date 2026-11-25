"""Tiện ích dùng chung cho các bước dựng scene trong `steps/`.

Các bước chạy được ở 2 nơi, cho ra cùng một file USD:
  * Script Editor của Isaac Sim (GUI): mở file bước, bấm Run.
  * `build_scene.py` (headless): chạy lần lượt các bước rồi lưu.

Điều kiện: stage hiện tại đã được lưu vào `sim/scenes/` (Save As) trước khi chạy bước 1,
để mọi đường dẫn tới assets được ghi dạng tương đối so với file scene.

Chỉ dùng pxr + omni.usd (không dùng API Isaac đã deprecated) để bền với Isaac Sim 6.x.
"""

from __future__ import annotations

import math
import os

import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdSemantics

SCENES_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(os.path.dirname(SCENES_DIR))
ROBOLAB_ASSETS = os.path.join(PROJECT_DIR, "robolab", "RoboLab", "assets")

# Franka Panda gốc (tay kẹp Panda, khớp với gripper `franka_panda` của GraspGenX).
# Tải từ máy chủ assets Isaac Sim 6.0 (cần Internet).
FRANKA_REL = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
FRANKA_FALLBACK_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"

def get_stage() -> Usd.Stage:
    stage = omni.usd.get_context().get_stage()
    if not stage.GetRootLayer().realPath:
        raise RuntimeError("Stage chưa được lưu. Hãy File > Save As vào sim/scenes/ trước khi chạy các bước.")
    return stage


def franka_url() -> str:
    try:
        from isaacsim.storage.native import get_assets_root_path

        root = get_assets_root_path()
    except Exception:  # extension chưa nạp hoặc không có mạng tới máy chủ kiểm tra
        root = None
    return (root or FRANKA_FALLBACK_ROOT) + FRANKA_REL


def robolab(rel_path: str) -> str:
    """Đường dẫn tuyệt đối tới một file trong robolab/RoboLab/assets/."""
    path = os.path.join(ROBOLAB_ASSETS, rel_path)
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    return path


def as_asset_path(stage: Usd.Stage, path: str) -> str:
    """File local -> đường dẫn tương đối so với file scene; URL giữ nguyên."""
    if "://" in path:
        return path
    base = os.path.dirname(stage.GetRootLayer().realPath)
    return os.path.relpath(path, base).replace(os.sep, "/")



def quat_from_yaw(yaw_deg: float) -> Gf.Quatd:
    h = math.radians(yaw_deg) / 2.0
    return Gf.Quatd(math.cos(h), 0.0, 0.0, math.sin(h))


def _precision(prim: Usd.Prim, name: str):
    """Asset được tham chiếu có thể đã khai báo op dạng float; phải ghi đè đúng kiểu đó."""
    attr = prim.GetAttribute(name)
    if attr and str(attr.GetTypeName()) in ("float3", "quatf"):
        return UsdGeom.XformOp.PrecisionFloat
    return UsdGeom.XformOp.PrecisionDouble


def set_pose(prim: Usd.Prim, xyz=(0.0, 0.0, 0.0), yaw_deg: float = 0.0, quat: Gf.Quatd | None = None,
             scale=(1.0, 1.0, 1.0)) -> None:
    """Ghi đè transform bằng đúng 3 op: translate, orient, scale (giống quy ước của RoboLab)."""
    F = UsdGeom.XformOp.PrecisionFloat
    q = quat if quat is not None else quat_from_yaw(yaw_deg)
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    p = _precision(prim, "xformOp:translate")
    x.AddTranslateOp(p).Set(Gf.Vec3f(*xyz) if p == F else Gf.Vec3d(*xyz))
    p = _precision(prim, "xformOp:orient")
    x.AddOrientOp(p).Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())) if p == F else q)
    p = _precision(prim, "xformOp:scale")
    x.AddScaleOp(p).Set(Gf.Vec3f(*scale) if p == F else Gf.Vec3d(*scale))


def add_reference(stage: Usd.Stage, prim_path: str, asset: str, xyz=(0.0, 0.0, 0.0), yaw_deg: float = 0.0) -> Usd.Prim:
    """Tạo (hoặc tạo lại) prim tham chiếu tới một file USD, đặt ở (xyz, yaw) trong hệ cha.

    Giữ nguyên transform gốc của asset (ví dụ grey_bin.usd có scale 0.007 vì mesh không tính bằng mét):
    transform cuối = transform của asset, rồi mới tới (xyz, yaw) của scene.
    Chạy lại bước nhiều lần vẫn cho cùng kết quả.
    """
    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(prim_path)
    prim = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
    prim.GetReferences().AddReference(as_asset_path(stage, asset))
    m_asset = UsdGeom.Xformable(prim).GetLocalTransformation()  # transform asset tự khai báo ở prim gốc
    m_scene = Gf.Matrix4d().SetRotate(quat_from_yaw(yaw_deg)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*xyz))
    tf = Gf.Transform()
    tf.SetMatrix(m_asset * m_scene)  # quy ước vector hàng của USD: áp m_asset trước, m_scene sau
    set_pose(prim, tuple(tf.GetTranslation()), quat=tf.GetRotation().GetQuat(), scale=tuple(tf.GetScale()))
    return prim


def add_label(prim: Usd.Prim, label: str) -> None:
    """Nhãn ngữ nghĩa (taxonomy "class") để camera xuất segmentation theo tên vật."""
    api = UsdSemantics.LabelsAPI.Apply(prim, "class")
    api.CreateLabelsAttr().Set([label])


def look_at_quat(eye, target, up=(0.0, 0.0, 1.0)) -> Gf.Quatd:
    """Hướng cho prim Camera USD (nhìn theo -Z, +Y lên) đặt ở `eye`, nhìn vào `target`."""
    eye, target, up = Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up)
    z = (eye - target).GetNormalized()  # trục +Z của camera hướng ngược chiều nhìn
    x = Gf.Cross(up, z).GetNormalized()
    y = Gf.Cross(z, x)
    m = Gf.Matrix3d(x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2])  # các hàng = trục camera trong hệ cha
    return m.ExtractRotation().GetQuat()


def define_camera(stage: Usd.Stage, prim_path: str, eye, target, up=(0.0, 0.0, 1.0), *, width: int, height: int,
                  fx: float, near: float = 0.05, far: float = 10.0) -> Usd.Prim:
    """Camera pinhole với nội tham số cho ảnh width x height (fx = fy, tâm ảnh ở giữa)."""
    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(prim_path)
    cam = UsdGeom.Camera.Define(stage, prim_path)
    h_ap = 20.955  # mm, mặc định USD; tiêu cự tính theo cùng đơn vị
    cam.CreateHorizontalApertureAttr(h_ap)
    cam.CreateVerticalApertureAttr(h_ap * height / width)
    cam.CreateFocalLengthAttr(fx * h_ap / width)
    cam.CreateClippingRangeAttr(Gf.Vec2f(near, far))
    cam.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    prim = cam.GetPrim()
    # Schema cảm biến RTX của Isaac Sim 6: cần để bọc prim bằng isaacsim.sensors.experimental.rtx.RtxCamera.
    if not prim.ApplyAPI("OmniSensorAPI"):
        prim.AddAppliedSchema("OmniSensorAPI")
    # Isaac Sim GUI tự ghi thêm các thông số phơi sáng này khi nhìn qua camera trên viewport. Ghi sẵn để file
    # dựng từ GUI hay headless đều giống nhau và ảnh render không phụ thuộc vào việc đã mở camera trên GUI chưa.
    for api in ("OmniRtxCameraAutoExposureAPI_1", "OmniRtxCameraExposureAPI_1"):
        prim.AddAppliedSchema(api)
    prim.CreateAttribute("exposure:fStop", Sdf.ValueTypeNames.Float).Set(5.0)
    prim.CreateAttribute("exposure:responsivity", Sdf.ValueTypeNames.Float).Set(1.1026709)
    prim.CreateAttribute("exposure:time", Sdf.ValueTypeNames.Float).Set(0.02)
    set_pose(prim, eye, quat=look_at_quat(eye, target, up))
    prim.SetCustomDataByKey("shimizu:resolution", Gf.Vec2i(width, height))
    return prim


def set_joint(stage: Usd.Stage, joint_path: str, value: float, *, angular: bool = True, drive: bool = True) -> None:
    """Đặt vị trí ban đầu (JointStateAPI) và đích drive cho một khớp. Góc tính bằng độ, khớp trượt bằng mét."""
    prim = stage.GetPrimAtPath(joint_path)
    if not prim:
        raise RuntimeError(f"Không thấy khớp {joint_path}")
    kind = "angular" if angular else "linear"
    PhysxSchema.JointStateAPI.Apply(prim, kind).CreatePositionAttr().Set(value)
    if drive:
        UsdPhysics.DriveAPI.Apply(prim, kind).CreateTargetPositionAttr().Set(value)
