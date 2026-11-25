"""Xuất quan sát RGB-D từ scene franka_pick_table theo định dạng `real_world` của GraspGenX, cho nhánh perception.

Mỗi quan sát là một thư mục gồm:
  rgb.png         uint8 HxWx3
  depth.npy       float32 HxW, mét, khoảng cách theo trục quang (z-depth), 0 = không hợp lệ
  seg.png         uint8 HxW, mask ground truth: 0 nền, 2 bàn, 3 robot, 4 giá kê robot, vật 101..106
  meta_data.json  intrinsics, camera_pose (= T_base_cam, camera -> robot base, quy ước OpenCV), label_map,
                  scene_bounds, obj_denylist (giống GraspGenX) + trường mở rộng: pose ground truth của vật, ...
Bố cục 0 là scene gốc. Các bố cục sau đặt lại 5 vật nhỏ ở vị trí/góc ngẫu nhiên (theo seed), thùng giữ nguyên.
Scene trên đĩa KHÔNG bị sửa: thay đổi vị trí chỉ nằm trong bộ nhớ của lần chạy này.

Chạy từ thư mục gốc dự án:
  OMNI_KIT_ACCEPT_EULA=Y ~/isaac-sim-6/python.sh sim/scenes/export_observations.py
  # tùy chọn: --out sim/exports/perception_rgbd_v1 --seeds 1 2 3
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
PROJECT = os.path.dirname(SIM)
parser = argparse.ArgumentParser()
parser.add_argument("--scene", default=os.path.join(HERE, "franka_pick_table.usda"))
parser.add_argument("--out", default=os.path.join(SIM, "exports", "perception_rgbd_v1"))
parser.add_argument("--seeds", nargs="*", type=int, default=[1, 2, 3])
parser.add_argument("--settle-s", type=float, default=1.0)
args, _ = parser.parse_known_args()
SCENE, OUT = os.path.abspath(args.scene), os.path.abspath(args.out)
os.makedirs(OUT, exist_ok=True)
T0 = time.time()


def log(msg: str) -> None:
    print(f"[export {time.time() - T0:6.1f}s] {msg}", flush=True)


from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import omni.kit.app  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.sensors.experimental.rtx", True)

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.prims import Articulation, RigidPrim  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

sys.path.insert(0, HERE)
import scene_lib as L  # noqa: E402

RES = (480, 640)  # (H, W)
ROBOT, HAND = "/World/franka", "/World/franka/panda_hand"
CAMERAS = {"table_cam": "/World/cameras/table_cam", "wrist_cam": "/World/franka/panda_hand/wrist_cam"}
FIXED = {"ground": 0, "table": 2, "robot": 3, "robot_stand": 4}
GROUP_PRIMS = {"table": "/World/env/table", "robot": ROBOT, "robot_stand": "/World/env/franka_table"}
SCENE_BOUNDS = [0.15, -0.55, -0.02, 0.95, 0.55, 0.60]  # xmin ymin zmin xmax ymax zmax, hệ robot base
CV = np.diag([1.0, -1.0, -1.0, 1.0])  # camera USD (-Z nhìn tới, +Y lên) -> OpenCV (+Z tới, +Y xuống)
# Vùng đặt ngẫu nhiên cho 5 vật nhỏ: trong tầm với, tránh thùng xám (y < -0.20) và mép bàn.
RAND_X, RAND_Y, MIN_DIST = (0.36, 0.70), (-0.10, 0.32), 0.13


def to_np(x) -> np.ndarray:
    return x.numpy() if hasattr(x, "numpy") else np.asarray(x)


def step(n: int = 1) -> None:
    for _ in range(n):
        app.update()


def gf_to_np(m) -> np.ndarray:
    """Gf.Matrix4d (quy ước vector hàng) -> ma trận 4x4 numpy quy ước cột."""
    return np.array(m, dtype=np.float64).T


def pose_to_T(pos, quat_wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat_wxyz)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [float(v) for v in pos]
    return T


def intrinsics(stage, path: str) -> np.ndarray:
    g = UsdGeom.Camera(stage.GetPrimAtPath(path))
    H, W = RES
    f = g.GetFocalLengthAttr().Get()
    ha, va = g.GetHorizontalApertureAttr().Get(), g.GetVerticalApertureAttr().Get()
    ho, vo = g.GetHorizontalApertureOffsetAttr().Get() or 0.0, g.GetVerticalApertureOffsetAttr().Get() or 0.0
    return np.array([[W * f / ha, 0, W / 2 + ho * W / ha], [0, H * f / va, H / 2 - vo * H / va], [0, 0, 1]])


def capture(sensor, max_frames: int = 120):
    for _ in range(max_frames):
        step()
        r, _ = sensor.get_data("rgb")
        d, _ = sensor.get_data("distance_to_image_plane")
        s, info = sensor.get_data("instance_id_segmentation")
        if r is None or d is None or s is None:
            continue
        rgb = to_np(r)[..., :3].astype(np.uint8)
        if rgb.mean() > 5:  # khung đầu của RTX có thể còn đen
            return rgb, to_np(d).reshape(RES).astype(np.float32), to_np(s).reshape(RES).astype(np.int64), info
    raise RuntimeError("annotator không trả dữ liệu")


def random_layout(rng, names, settled_z):
    """Vị trí (x, y) ngẫu nhiên cách nhau >= MIN_DIST, góc yaw ngẫu nhiên; z giữ độ cao nằm yên của bố cục gốc."""
    for _ in range(2000):
        xy = [(rng.uniform(*RAND_X), rng.uniform(*RAND_Y)) for _ in names]
        if all(math.dist(a, b) >= MIN_DIST for i, a in enumerate(xy) for b in xy[i + 1:]):
            return {n: ((x, y, settled_z[n]), float(rng.uniform(0, 360))) for n, (x, y) in zip(names, xy)}
    raise RuntimeError("không tìm được bố cục ngẫu nhiên")


# ----------------------------------------------------------------------------- mở scene
ctx = omni.usd.get_context()
ctx.open_stage(SCENE)
step(10)
while ctx.get_stage_loading_status()[2] > 0:
    step()
stage = ctx.get_stage()
objects = {}
for prim in stage.GetPrimAtPath("/World/objects").GetChildren():
    objects[prim.GetName()] = dict(path=str(prim.GetPath()), seg_id=int(prim.GetCustomDataByKey("shimizu:seg_id")),
                                  aliases=list(prim.GetCustomDataByKey("shimizu:aliases") or []),
                                  source=prim.GetCustomDataByKey("shimizu:source"))
movable = [n for n in objects if n != "grey_bin"]
settled_z = {n: float(stage.GetPrimAtPath(o["path"]).GetAttribute("xformOp:translate").Get()[2])
             for n, o in objects.items()}
default_pose = {n: (tuple(stage.GetPrimAtPath(objects[n]["path"]).GetAttribute("xformOp:translate").Get()),
                    stage.GetPrimAtPath(objects[n]["path"]).GetAttribute("xformOp:orient").Get()) for n in movable}
label_map = dict(FIXED, **{n: o["seg_id"] for n, o in objects.items()})
groups = [(GROUP_PRIMS[g], FIXED[g]) for g in ("table", "robot", "robot_stand")] + \
         [(o["path"], o["seg_id"]) for o in objects.values()]

sensors = {c: CameraSensor(RtxCamera(p), resolution=RES,
                           annotators=["rgb", "distance_to_image_plane", "instance_id_segmentation"])
           for c, p in CAMERAS.items()}
Ks = {c: intrinsics(stage, p) for c, p in CAMERAS.items()}
T_hand_wrist = gf_to_np(UsdGeom.Xformable(stage.GetPrimAtPath(CAMERAS["wrist_cam"])).GetLocalTransformation())
timeline = omni.timeline.get_timeline_interface()
bodies = robot = hand = None

layouts = [("L0", None)] + [(f"L{i + 1}", s) for i, s in enumerate(args.seeds)]
manifest = dict(scene=os.path.relpath(SCENE, PROJECT), isaac_sim="6.0.1", created=time.strftime("%Y-%m-%d %H:%M"),
                label_map=label_map, observations=[], checks={})

for lname, seed in layouts:
    # ------------------------------------------------------------- đặt bố cục (chỉ trong bộ nhớ)
    if seed is None:
        poses = {n: (default_pose[n][0], None) for n in movable}
        for n in movable:
            L.set_pose(stage.GetPrimAtPath(objects[n]["path"]), default_pose[n][0], quat=default_pose[n][1])
    else:
        poses = random_layout(np.random.default_rng(seed), movable, settled_z)
        for n, (xyz, yaw) in poses.items():
            L.set_pose(stage.GetPrimAtPath(objects[n]["path"]), xyz, yaw_deg=yaw)
    step(2)
    # Hộp bao của vật ở tư thế khai báo (khung hộp -> world), dùng để kiểm tra điểm chiếu ngược.
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    boxes = {}
    for n, o in objects.items():
        b = cache.ComputeWorldBound(stage.GetPrimAtPath(o["path"]))
        boxes[n] = (gf_to_np(b.GetMatrix()), np.array(b.GetRange().GetMin()), np.array(b.GetRange().GetMax()))

    # ------------------------------------------------------------- mô phỏng cho vật nằm yên
    timeline.play()
    step(3)
    if bodies is None:
        bodies = RigidPrim([o["path"] for o in objects.values()])
        robot = Articulation(ROBOT)
        # KHÔNG bọc panda_hand bằng RigidPrim: làm vậy gây nhiễu mô phỏng (bàn bị đẩy lên 11.5 mm, đã đo).
        # Đọc tư thế các khâu qua view articulation của PhysX: mỗi khâu [x, y, z, qx, qy, qz, qw].
        link_names = list(robot._physics_articulation_view.shared_metatype.link_names)
        hand_idx = link_names.index(HAND.rsplit("/", 1)[1])
    P0 = [pose_to_T(p, q) for p, q in zip(*(to_np(a) for a in bodies.get_world_poses()))]
    t_start = SimulationManager.get_simulation_time()
    while SimulationManager.get_simulation_time() - t_start < args.settle_s:
        step()

    shots = {c: capture(s) for c, s in sensors.items()}
    P1 = [pose_to_T(p, q) for p, q in zip(*(to_np(a) for a in bodies.get_world_poses()))]
    bp, bq = (to_np(a)[0] for a in robot.get_world_poses())
    T_world_base = pose_to_T(bp, bq)
    lt = to_np(robot._physics_articulation_view.get_link_transforms()).reshape(-1, len(link_names), 7)[0, hand_idx]
    T_world_hand = pose_to_T(lt[:3], [lt[6], lt[3], lt[4], lt[5]])
    q = to_np(robot.get_dof_positions())[0]
    sim_t = SimulationManager.get_simulation_time()
    T_base_obj = {n: np.linalg.inv(T_world_base) @ P1[i] for i, n in enumerate(objects)}

    for cam, (rgb, depth, inst, info) in shots.items():
        oid = f"{lname}_{cam}"
        d = os.path.join(OUT, oid)
        os.makedirs(d, exist_ok=True)
        depth[~np.isfinite(depth)] = 0.0
        depth[(depth <= 0) | (depth > 10.0)] = 0.0
        id_to_labels = info.get("idToLabels", {}) if isinstance(info, dict) else {}
        seg = np.zeros(RES, np.uint8)
        for prefix, k in groups:
            for i, p in id_to_labels.items():
                if str(p) == prefix or str(p).startswith(prefix + "/"):
                    seg[inst == int(i)] = k
        if cam == "table_cam":
            T_world_cam = gf_to_np(UsdGeom.Xformable(stage.GetPrimAtPath(CAMERAS[cam])).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default())) @ CV
        else:  # camera cổ tay đi theo panda_hand: lấy tư thế tay từ mô phỏng, không lấy từ USD
            T_world_cam = T_world_hand @ T_hand_wrist @ CV
        T_base_cam = np.linalg.inv(T_world_base) @ T_world_cam
        K = Ks[cam]

        # --------------------------------------------------------- kiểm tra quy ước camera bằng số
        H, W = RES
        v, u = np.mgrid[0:H, 0:W]
        pts_cam = np.stack([(u - K[0, 2]) * depth / K[0, 0], (v - K[1, 2]) * depth / K[1, 1], depth, np.ones(RES)], -1)
        pts_base = pts_cam.reshape(-1, 4) @ T_base_cam.T
        checks, visible = {}, []
        for i, n in enumerate(objects):
            mask = (seg == objects[n]["seg_id"]).reshape(-1) & (depth.reshape(-1) > 0)
            npx = int(mask.sum())
            if npx < 50:
                checks[n] = dict(pixels=npx, visible=False)
                continue
            visible.append(n)
            M, lo, hi = boxes[n]
            T_box = (np.linalg.inv(T_world_base) @ P1[i] @ np.linalg.inv(P0[i]) @ M)  # khung hộp -> base, lúc chụp
            pb = pts_base[mask] @ np.linalg.inv(T_box).T
            margin = 0.01 / np.linalg.norm(T_box[:3, :3], axis=0)  # lề 1 cm quy về đơn vị của khung hộp (asset có scale)
            inside = np.all((pb[:, :3] >= lo - margin) & (pb[:, :3] <= hi + margin), axis=1).mean()
            c_base = T_box @ np.r_[(lo + hi) / 2, 1.0]
            c_cam = np.linalg.inv(T_base_cam) @ c_base
            uu, vv = K[0, 0] * c_cam[0] / c_cam[2] + K[0, 2], K[1, 1] * c_cam[1] / c_cam[2] + K[1, 2]
            in_img = 0 <= uu < W and 0 <= vv < H
            near = in_img and (seg[max(0, int(vv) - 3):int(vv) + 4, max(0, int(uu) - 3):int(uu) + 4]
                               == objects[n]["seg_id"]).any()
            # Tâm không rơi vào mask vẫn hợp lệ nếu vật bị cắt ở mép ảnh, hoặc có vật khác che phía trước tâm.
            occluded = in_img and 0 < depth[int(vv), int(uu)] < c_cam[2] - 0.01
            explained = near or not in_img or occluded
            checks[n] = dict(pixels=npx, visible=True, frac_points_in_box=round(float(inside), 4),
                             center_projects_into_mask=bool(near), center_outside_image=not in_img,
                             center_occluded=bool(occluded), center_check_ok=bool(explained),
                             centroid_offset_m=round(float(np.linalg.norm(pts_base[mask][:, :3].mean(0) - c_base[:3])), 4))
        ok = all(c["frac_points_in_box"] >= 0.95 for c in checks.values() if c["visible"]) and \
            all(c["center_check_ok"] for c in checks.values() if c["visible"])
        manifest["checks"][oid] = dict(pass_=ok, per_object=checks)

        # --------------------------------------------------------- ghi file
        Image.fromarray(rgb).save(os.path.join(d, "rgb.png"))
        np.save(os.path.join(d, "depth.npy"), depth)
        Image.fromarray(seg).save(os.path.join(d, "seg.png"))
        meta = dict(
            intrinsics=K.tolist(), camera_pose=T_base_cam.tolist(), label_map=label_map, scene_bounds=SCENE_BOUNDS,
            obj_denylist=[n for n in objects if not checks[n]["visible"]],
            obj_denylist_reason="not visible in this view (fewer than 50 pixels)",
            # --- trường mở rộng (không có trong GraspGenX gốc)
            frame_id="panda_link0", camera_name=cam, camera_prim=CAMERAS[cam], source="isaac_sim",
            scene=os.path.relpath(SCENE, PROJECT), layout=lname, layout_seed=seed, stamp=round(sim_t, 4),
            depth_type="z", depth_unit="m", camera_convention="opencv", resolution=[W, H],
            gt_prim_paths={n: o["path"] for n, o in objects.items()},
            gt_object_poses={n: T.round(6).tolist() for n, T in T_base_obj.items()},
            object_aliases={n: o["aliases"] for n, o in objects.items()},
            T_world_base=T_world_base.round(6).tolist(), T_base_hand=(np.linalg.inv(T_world_base) @ T_world_hand).round(6).tolist(),
            robot_joint_positions=[round(float(x), 5) for x in q],
        )
        json.dump(meta, open(os.path.join(d, "meta_data.json"), "w"), indent=2)
        manifest["observations"].append(dict(id=oid, camera=cam, layout=lname, seed=seed, visible_objects=visible,
                                             depth_valid_frac=round(float((depth > 0).mean()), 4)))
        log(f"{'OK  ' if ok else 'FAIL'} {oid}: thấy {len(visible)} vật "
            + ", ".join(f"{n}={checks[n]['frac_points_in_box']}" for n in visible))
    timeline.stop()
    step(3)

n_ok = sum(c["pass_"] for c in manifest["checks"].values())
manifest["summary"] = f"{n_ok}/{len(manifest['checks'])} observations pass camera-convention checks"
json.dump(manifest, open(os.path.join(OUT, "manifest.json"), "w"), indent=2, ensure_ascii=False)
log(manifest["summary"] + f" -> {os.path.relpath(OUT, PROJECT)}")

# Ảnh xem nhanh: mỗi hàng một quan sát (RGB | depth | seg).
PAL = np.zeros((256, 3), np.uint8)
PAL[[2, 3, 4]] = [[200, 60, 80], [60, 180, 75], [120, 120, 120]]
PAL[101:107] = [[255, 225, 25], [0, 130, 200], [245, 130, 48], [145, 30, 180], [70, 240, 240], [240, 50, 230]]
rows = []
for o in manifest["observations"]:
    d = os.path.join(OUT, o["id"])
    rgb = Image.open(os.path.join(d, "rgb.png"))
    dep = np.load(os.path.join(d, "depth.npy"))
    seg = np.array(Image.open(os.path.join(d, "seg.png")))
    v = dep > 0
    roi = v & ((seg == FIXED["table"]) | (seg >= 101))  # dải màu theo bàn + vật, bỏ qua ngón kẹp và sàn
    lo_, hi_ = np.percentile(dep[roi], [1, 99]) if roi.any() else (0, 1)
    vis = np.zeros(RES, np.uint8)
    vis[v] = (255 * (1 - np.clip((dep[v] - lo_) / max(hi_ - lo_, 1e-6), 0, 1))).astype(np.uint8)
    row = Image.new("RGB", (640 * 3, 480))
    for i, im in enumerate((rgb, Image.fromarray(vis).convert("RGB"), Image.fromarray(PAL[seg]))):
        row.paste(im, (640 * i, 0))
    rows.append(row.resize((960, 240)))
sheet = Image.new("RGB", (960, 240 * len(rows)))
for i, r in enumerate(rows):
    sheet.paste(r, (0, 240 * i))
sheet.save(os.path.join(OUT, "preview.png"))
app.close()
sys.exit(0 if n_ok == len(manifest["checks"]) else 1)
