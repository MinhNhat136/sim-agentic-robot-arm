# Bộ quan sát RGB-D từ Isaac Sim cho nhánh perception (v1)

> Tạo ngày 2026-11-08 bằng `sim/scenes/export_observations.py` từ scene `sim/scenes/franka_pick_table.usda` (Isaac Sim 6.0.1).
> Định dạng giống `real_world` của GraspGenX (giống fixture `assets/sample_data/real_world/`), nên dùng lại được code `graspgenx/tutorials/bai4_camera_rgbd.py`.
> Nhãn **[XM]** = đã kiểm bằng số.

## 1. Nội dung

8 quan sát, gồm 4 cách bày vật × 2 camera. Bố cục L0 là scene gốc. L1–L3 đặt lại 5 vật nhỏ ở vị trí và góc xoay ngẫu nhiên (seed 1–3), thùng xám giữ nguyên. Mỗi bố cục được mô phỏng 1 s cho vật nằm yên rồi mới chụp.

| Thư mục | Camera | Seed | Vật nhìn thấy |
|---|---|---|---|
| `L0_table_cam` | table_cam | gốc | cả 6 vật |
| `L0_wrist_cam` | wrist_cam | gốc | mustard_bottle, red_mug, tomato_soup_can, grey_bin |
| `L1_table_cam` | table_cam | 1 | cả 6 vật |
| `L1_wrist_cam` | wrist_cam | 1 | mustard_bottle\*, red_block, blue_block, tomato_soup_can\*, grey_bin |
| `L2_table_cam` | table_cam | 2 | cả 6 vật |
| `L2_wrist_cam` | wrist_cam | 2 | red_block, blue_block\*, tomato_soup_can, grey_bin |
| `L3_table_cam` | table_cam | 3 | cả 6 vật |
| `L3_wrist_cam` | wrist_cam | 3 | mustard_bottle\*, red_mug\*, blue_block, grey_bin |

\* bị cắt ở mép ảnh. `preview.png` xếp mỗi hàng một quan sát theo thứ tự trên (RGB | depth | seg). `manifest.json` chứa danh sách quan sát và kết quả kiểm tra.

- **table_cam**: camera cố định đối diện robot, đặt ở (1.35, 0, 0.80) m, nhìn xuống bàn. fx = fy = 615 px, cx = 320, cy = 240.
- **wrist_cam**: camera gắn trên `panda_hand`, robot ở tư thế ready. fx = fy = 340 px. Thấy hai đầu ngón kẹp ở mép dưới ảnh. Vật ở xa (x ≈ 0.66 m) thường nằm ngoài khung.

## 2. Định dạng mỗi quan sát

| File | Nội dung |
|---|---|
| `rgb.png` | uint8, 480×640×3 |
| `depth.npy` | float32, 480×640, **mét**, khoảng cách theo trục quang (z-depth, annotator `distance_to_image_plane`), 0 = không hợp lệ |
| `seg.png` | uint8, 480×640, mask ground truth theo `label_map` |
| `meta_data.json` | xem bên dưới |

**Trường giống GraspGenX:**
- `intrinsics`: K 3×3.
- `camera_pose`: **T_base_cam** 4×4, biến đổi từ hệ camera sang hệ robot base, camera theo quy ước **OpenCV** (+Z nhìn tới, +X sang phải, +Y xuống).
- `label_map`: `ground 0, table 2, robot 3, robot_stand 4, mustard_bottle 101, red_mug 102, red_block 103, blue_block 104, tomato_soup_can 105, grey_bin 106`.
- `scene_bounds`: `[xmin, ymin, zmin, xmax, ymax, zmax] = [0.15, -0.55, -0.02, 0.95, 0.55, 0.60]`, hệ base.
- `obj_denylist`: **ở đây là các vật không nhìn thấy trong khung hình (< 50 px)**. Mask trong sim luôn đúng, nên khác với fixture thật, nơi denylist dùng cho vật có mask xấu.

**Trường mở rộng:**
- Thông tin chụp: `frame_id` (`panda_link0`), `camera_name`, `camera_prim`, `source`, `scene`, `layout`, `layout_seed`, `stamp`, `depth_type` (`z`), `depth_unit` (`m`), `camera_convention` (`opencv`), `resolution`.
- Ground truth của vật: `gt_prim_paths`; `gt_object_poses` là T_base_obj 4×4 của từng vật lúc chụp, gốc là gốc prim của asset; `object_aliases` là tên gọi tiếng Anh, ví dụ `red mug`, `red cup`.
- Trạng thái robot: `T_world_base` (đơn vị, vì robot đặt ở gốc world), `T_base_hand` (tư thế `panda_hand` lúc chụp), `robot_joint_positions` (9 giá trị: 7 khớp tính bằng rad, 2 ngón tính bằng m).

**Hệ tọa độ:** gốc robot `panda_link0` trùng gốc world, +X ra trước, +Y sang trái, +Z lên. Mặt bàn ở z ≈ 0.003 m: bàn là vật thể động, chân bàn được khai báo thấp hơn sàn 3 mm nên khi mô phỏng cả bàn bị đẩy lên 3 mm.

**Lưu ý với `grey_bin`:** `gt_object_poses` chỉ có vị trí và hướng, không chứa scale 0.007 mà asset `grey_bin.usd` tự khai báo ở prim gốc. Nếu dựng lại điểm ground truth từ mesh của thùng bằng pose này, phải nhân thêm scale 0.007. Các vật khác có scale 1.

## 3. Đọc dữ liệu và tạo point cloud của một vật (hệ robot base)

```python
import json, numpy as np
from PIL import Image

d = "sim/exports/perception_rgbd_v1/L0_table_cam"
meta = json.load(open(f"{d}/meta_data.json"))
K, T_base_cam = np.array(meta["intrinsics"]), np.array(meta["camera_pose"])
depth, seg = np.load(f"{d}/depth.npy"), np.array(Image.open(f"{d}/seg.png"))

v, u = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
xyz_cam = np.stack([(u - K[0, 2]) * depth / K[0, 0], (v - K[1, 2]) * depth / K[1, 1], depth], -1)
mask = (seg == meta["label_map"]["mustard_bottle"]) & (depth > 0)
pts_cam = xyz_cam[mask]                                    # (N, 3), hệ camera OpenCV
pts_base = pts_cam @ T_base_cam[:3, :3].T + T_base_cam[:3, 3]  # (N, 3), hệ robot base: đưa thẳng cho GraspGenX
```

Với GraspGenX trên server: theo code đọc dữ liệu của `bai4_camera_rgbd.py` (dùng `intrinsics`, `camera_pose`, `label_map`, `scene_bounds`, `depth.npy`, `seg.png`), lệnh `bai4_camera_rgbd.py --scene <thư mục quan sát> --object mustard_bottle` sẽ đọc được bộ này **[CXM: chưa chạy thử trên server]**.

## 4. Kiểm chứng [XM]

`manifest.json`: **8/8 quan sát đạt**. Với mỗi vật nhìn thấy:
1. Chiếu ngược depth của các pixel trong mask sang hệ base: **100% điểm nằm trong hộp bao thật của vật** (lấy từ USD và pose mô phỏng lúc chụp), lề 1 cm. Kiểm tra này gồm cả `T_base_cam` của wrist_cam, vốn tính từ tư thế khâu tay trong mô phỏng.
2. Chiếu tâm hộp bao của vật lên ảnh: tâm rơi vào mask của vật. Các trường hợp không rơi vào mask đều là vật bị cắt ở mép ảnh (tâm nằm ngoài khung), đánh dấu \* ở §1.
3. Tâm đám điểm lệch tâm vật 1–4 cm. Đây là điều bình thường vì camera chỉ thấy mặt trước của vật (point cloud partial).
4. Ở bố cục L0, `gt_object_poses` có độ cao trùng vị trí khai báo trong scene (ví dụ chai mù tạt z = 0.0823 m).

## 5. Khác biệt so với dữ liệu thật (cần lưu ý khi thử nghiệm)

- **Depth hoàn hảo:** không nhiễu, không lỗ, 100% pixel hợp lệ, kể cả mép vật và bề mặt bóng. RealSense thật có nhiễu, lỗ ở mép và mất dữ liệu trên vật trong suốt hoặc tối. Muốn thử độ bền thì nên tự thêm nhiễu.
- **RGB render:** không nhiễu, không mờ chuyển động, ánh sáng cố định (DomeLight + DistantLight).
- **Độ phân giải 640×480**, khác fixture thật của GraspGenX (1280×720). K luôn đọc từ `meta_data.json`, không giả định.
- **Mask ground truth chính xác tới pixel** (instance segmentation của renderer). Dùng làm chuẩn để chấm detector open-vocab, không dùng làm đầu vào khi đánh giá.

## 6. Tạo lại / mở rộng

```bash
OMNI_KIT_ACCEPT_EULA=Y ~/isaac-sim-6/python.sh sim/scenes/export_observations.py --out sim/exports/perception_rgbd_v1 --seeds 1 2 3
```

Mất khoảng 30 s. Thêm seed để có thêm bố cục. Scene trên đĩa không bị sửa: các bố cục ngẫu nhiên chỉ nằm trong bộ nhớ.
