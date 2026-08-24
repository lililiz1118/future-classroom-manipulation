#!/home/yuan/robot_ssh/.conda/envs/global_3.10/bin/python3
# -*- coding: utf-8 -*-
"""
Phase 0.4: 手眼标定验证

原理:
  放置 ArUco 标记 → freedrive 触碰标记中心 → 记录基座坐标
  → 相机拍摄标记 → 通过手眼标定转换到基座坐标
  → 对比两者差距 = 标定误差

依赖: opencv-contrib-python (ArUco)

用法:
  python3 verify_hand_eye.py              # 单点验证
  python3 verify_hand_eye.py --rounds 3   # 多点验证

准备工作:
  打印一个 ArUco 标记（推荐 DICT_4X4_50, ID=0, 边长≥5cm），贴在硬纸板上
"""

import sys
import os
import time
import json
from datetime import datetime

# ── math3d monkey-patch ──
import math3d.vector as _mv
_orig_truediv = _mv.Vector.__truediv__

def _patched_truediv(self, other):
    if _mv.utils.is_num_type(other):
        if _mv.np.isclose(other, 0.0):
            raise ZeroDivisionError("In division of vector by scalar")
        return _mv.Vector(1.0 / other * self._data)
    else:
        raise _mv.utils.Error("__truediv__ : Could not divide by non-number")

_mv.Vector.__truediv__ = _patched_truediv
_mv.Vector.__div__ = _patched_truediv
# ─────────────────────────

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '../../my_ur_control/scripts'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '../../my_ur_control/scripts/ur_control'))

import urx
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import collections.abc
collections.Iterable = collections.abc.Iterable

from gripper import AG95NoInit

# ── ROS imports ──
import rospy
import tf.transformations as tft
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as ImageMsg

# ── 参数 ─────────────────────────────────────────────────
ROBOT_IP = "192.168.131.3"
HOME_J = [1.60624647, -2.69281799, 2.24398565, -2.31487161, -1.60637647, 0.00013183]

# ⚠️ 待验证的手眼标定值（相机在末端执行器下的位姿）
#   来源: 旧项目 ur3_grab.py (T_wrist3_link → d405_color_optical_frame)
EE_CAM_XYZ_OLD = np.array([-0.009597337799696761, -0.07408851538404479, 0.01670505075735617])
EE_CAM_Q_OLD   = np.array([0.0006344396200247934, -6.58198103146157e-05,
                       0.006961144575743825, 0.999975567511685])
#   新标定结果 (calibrate_hand_eye.py 输出):
EE_CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              '../config/hand_eye_calibration_new.json')
EE_CAM_XYZ = EE_CAM_XYZ_OLD
EE_CAM_Q = EE_CAM_Q_OLD

# ArUco 参数
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
MARKER_SIZE = 0.05  # 标记边长 (m)，根据实际打印尺寸修改

# 相机内参（从 /d405/color/camera_info 获取，或手动设置）
CAMERA_INFO_TOPIC = "/d405/color/camera_info"
# 直接订阅图像 topic，手动 numpy 解码（完全不依赖 cv_bridge/libffi）
COLOR_TOPIC = "/d405/color/image_raw"
DEPTH_TOPIC = "/d405/depth/image_rect_raw"

# 结果保存
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_FILE = os.path.join(SCRIPT_DIR, '../config/hand_eye_verify.json')


def fmt_arr(values, width=8):
    inner = ", ".join(f"{v:{width}.4f}" for v in values)
    return f"[{inner}]"


# ═══════════════════════════════════════════════════════
#  变换函数 (复用 ur3_grab.py 逻辑)
# ═══════════════════════════════════════════════════════

def transform_target_to_ee(t_cam_ee, q_cam_ee, t_target_cam, q_target_cam):
    """目标从相机系 → 末端系"""
    R_cam_ee = R.from_quat(q_cam_ee).as_matrix()
    t_cam_ee = np.array(t_cam_ee)
    R_target_cam = R.from_quat(q_target_cam).as_matrix()
    t_target_cam = np.array(t_target_cam)
    R_target_ee = R_cam_ee @ R_target_cam
    t_target_ee = R_cam_ee @ t_target_cam + t_cam_ee
    q_target_ee = R.from_matrix(R_target_ee).as_quat()
    return t_target_ee, q_target_ee


def transform_ee_to_base(robot, target_ee_pos, target_ee_quat):
    """目标从末端系 → 基座系 (使用当前 TCP 位姿)"""
    ee_pose = robot.getl()
    ee_pos = np.array(ee_pose[:3])
    ee_rot = R.from_rotvec(ee_pose[3:])

    T_base_ee = np.eye(4)
    T_base_ee[:3, :3] = ee_rot.as_matrix()
    T_base_ee[:3, 3] = ee_pos

    T_ee_target = np.eye(4)
    T_ee_target[:3, :3] = R.from_quat(target_ee_quat).as_matrix()
    T_ee_target[:3, 3] = target_ee_pos

    T_base_target = T_base_ee @ T_ee_target
    pos = T_base_target[:3, 3]
    quat = R.from_matrix(T_base_target[:3, :3]).as_quat()
    return pos, quat


# ═══════════════════════════════════════════════════════
#  相机内参获取
# ═══════════════════════════════════════════════════════

_camera_matrix = None
_dist_coeffs = None


def get_camera_intrinsics():
    """从 ROS topic 获取相机内参，失败则用默认值"""
    global _camera_matrix, _dist_coeffs
    if _camera_matrix is not None:
        return _camera_matrix, _dist_coeffs

    try:
        msg = rospy.wait_for_message(CAMERA_INFO_TOPIC, CameraInfo, timeout=3.0)
        _camera_matrix = np.array(msg.K).reshape(3, 3)
        _dist_coeffs = np.array(msg.D)
        print(f"✅ 相机内参 (from ROS): fx={_camera_matrix[0,0]:.1f}, "
              f"fy={_camera_matrix[1,1]:.1f}")
    except Exception:
        # D405 默认内参 (1280x720)
        print("⚠️  无法获取 ROS camera_info，使用 D405 默认内参")
        _camera_matrix = np.array([
            [643.0, 0, 643.0],
            [0, 643.0, 363.0],
            [0, 0, 1]
        ])
        _dist_coeffs = np.zeros(5)
        print(f"  默认: fx=643, fy=643, cx=643, cy=363")
    return _camera_matrix, _dist_coeffs


# ═══════════════════════════════════════════════════════
#  ArUco 检测
# ═══════════════════════════════════════════════════════

def detect_aruco_marker(rgb_image, target_id):
    """
    检测指定 ID 的 ArUco 标记，返回其在相机坐标系下的 3D 位置
    参数: target_id — 要查找的标记 ID
    返回: (success, marker_xyz_cam, marker_quat_cam, corner_points, detected_ids)
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    corners, ids, _ = detector.detectMarkers(rgb_image)
    detected_ids = ids.flatten().tolist() if ids is not None else []

    if ids is None or target_id not in detected_ids:
        return False, None, None, None, detected_ids

    idx = list(detected_ids).index(target_id)
    corner = corners[idx][0]  # 4 个角点 [[x,y], ...]

    # 用 solvePnP 估计标记在相机系下的位姿
    cam_mat, dist = get_camera_intrinsics()
    half = MARKER_SIZE / 2
    obj_points = np.array([
        [-half, -half, 0],
        [ half, -half, 0],
        [ half,  half, 0],
        [-half,  half, 0],
    ], dtype=np.float32)

    success, rvec, tvec = cv2.solvePnP(obj_points, corner, cam_mat, dist)
    if not success:
        return False, None, None, None, detected_ids

    # 标记中心在相机系下的位置
    marker_xyz = tvec.flatten()
    marker_quat = R.from_rotvec(rvec.flatten()).as_quat()  # [x,y,z,w]

    return True, marker_xyz, marker_quat, corner, detected_ids


# ═══════════════════════════════════════════════════════
#  图像捕获
# ═══════════════════════════════════════════════════════

def rosimg_to_numpy(msg):
    """手动将 ROS Image 消息解码为 numpy 数组 (不依赖 cv_bridge，避免 conda libffi 冲突)"""
    # 编码 → numpy dtype 映射
    encoding_map = {
        'bgr8':    (np.uint8, 3),
        'rgb8':    (np.uint8, 3),
        'bgra8':   (np.uint8, 4),
        'rgba8':   (np.uint8, 4),
        'mono8':   (np.uint8, 1),
        '16UC1':   (np.uint16, 1),
        '32FC1':   (np.float32, 1),
    }
    dtype, channels = encoding_map.get(msg.encoding, (np.uint8, 3))
    arr = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
    # OpenCV 需要 BGR，如果是 RGB 则转换
    if msg.encoding == 'rgb8':
        arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    elif msg.encoding == 'rgba8':
        arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    elif msg.encoding == 'bgra8':
        arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    elif msg.encoding == 'mono8':
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    return arr


def capture_images(timeout=5.0):
    """直接从 ROS topic 订阅图像，手动 numpy 解码 (完全不依赖 cv_bridge/librealsense)"""
    # wait_for_message 接收的是显式的 topic 名 + 消息类型
    color_msg = rospy.wait_for_message(COLOR_TOPIC, ImageMsg, timeout=timeout)
    depth_msg = rospy.wait_for_message(DEPTH_TOPIC, ImageMsg, timeout=timeout)

    if color_msg is None:
        raise RuntimeError(f"未收到 color 图像: {COLOR_TOPIC}")
    if depth_msg is None:
        raise RuntimeError(f"未收到 depth 图像: {DEPTH_TOPIC}")

    rgb = rosimg_to_numpy(color_msg)
    depth = rosimg_to_numpy(depth_msg)
    return rgb, depth


# ═══════════════════════════════════════════════════════
#  主流程
# ═══════════════════════════════════════════════════════

def main():
    import argparse
    parser = argparse.ArgumentParser(description="手眼标定验证")
    parser.add_argument("--rounds", type=int, default=1,
                        help="验证点数 (默认 1)")
    parser.add_argument("--marker-size", type=float, default=MARKER_SIZE,
                        help=f"ArUco 标记边长 (m), 默认 {MARKER_SIZE}")
    parser.add_argument("--tcp-offset", type=float, nargs=3, default=[0.0, 0.0, 0.13],
                        metavar=('DX', 'DY', 'DZ'),
                        help="闭合指尖中心在 flange 系下的偏移 (m), "
                             "手指沿法兰 Z+ 向外为正, 默认 [0, 0, 0.13] (DH AG-160-95 闭合指尖)"
                        )
    args = parser.parse_args()

    marker_size = args.marker_size
    tcp_offset = np.array(args.tcp_offset)

    # 尝试加载新标定结果
    if os.path.exists(EE_CALIB_FILE):
        with open(EE_CALIB_FILE) as f:
            new_calib = json.load(f)
        global EE_CAM_XYZ, EE_CAM_Q
        EE_CAM_XYZ = np.array(new_calib['ee_cam_xyz'])
        EE_CAM_Q = np.array(new_calib['ee_cam_q'])
        print(f"📌 使用新标定: {EE_CALIB_FILE}")
        print(f"   采样数: {new_calib.get('num_samples', '?')}, "
              f"时间: {new_calib.get('timestamp', '?')}")

    # ═══ 连接 ═══════════════════════════════════════════
    print("=" * 60)
    print("  Phase 0.4: 手眼标定验证")
    print("=" * 60)

    # 初始化 ROS（disable_signals 防止 Ctrl+C/freedrive 退出导致 ROS 关闭）
    rospy.init_node("verify_hand_eye", anonymous=True, disable_signals=True)
    print("✅ ROS 就绪")

    # 获取相机内参
    get_camera_intrinsics()

    print(f"\n连接 UR 机械臂 {ROBOT_IP} ...")
    robot = urx.Robot(ROBOT_IP)
    data = robot.secmon.get_all_data()
    mode = data.get('RobotModeData', {}).get('robotMode', -1)
    if mode != 7:
        print(f"❌ 机械臂未就绪 (mode={mode}, 需要 RUNNING=7)")
        robot.close()
        sys.exit(1)
    print("✅ 机械臂就绪")

    # 连接夹爪
    print("\n连接 DH 夹爪...")
    gripper = AG95NoInit(port="/dev/dh_gripper_usb")
    gripper.initialize()
    print(f"✅ 夹爪就绪 (位置: {gripper.read_pos()})")

    # 验证 TCP 偏移近似值
    if tcp_offset[2] != 0:
        print(f"\n⚠️  TCP 偏移: flange → 闭合指尖 = [{tcp_offset[0]:.4f}, "
              f"{tcp_offset[1]:.4f}, {tcp_offset[2]:.4f}] m")
        print("   XY 误差直接反映手眼标定精度")
        print("   Z  误差 = 手眼误差 + TCP 偏移估计误差")
        print("   如需更精确的 Z，请用 --tcp-offset DX DY DZ 手动指定")

    # 回 home
    print("\n→ 回 home...")
    robot.movej(HOME_J, acc=0.5, vel=0.5)
    time.sleep(0.5)

    results = []

    for r in range(args.rounds):
        target_id = r  # 第1轮→ID 0, 第2轮→ID 1, ...

        print(f"\n{'─' * 60}")
        print(f"  第 {r+1}/{args.rounds} 个验证点  (ArUco ID={target_id})")
        print(f"{'─' * 60}")

        # ─── Step 1: 闭合夹爪 + Freedrive 触碰 ───
        print(f"\n  [Step 1] 闭合夹爪 → 触碰标记 ID={target_id} 中心")
        print("    夹爪闭合 → 指尖形成清晰矩形的几何中心")
        gripper.set_pos(0)      # 全闭
        time.sleep(1.5)
        print("    ✅ 夹爪已闭合，指尖中心 ≈ flange 正下方")
        if tcp_offset[2] != 0:
            print(f"    TCP 偏移 (flange→指尖): [{tcp_offset[0]:.4f}, "
                  f"{tcp_offset[1]:.4f}, {tcp_offset[2]:.4f}] m")

        print("\n    把 ArUco 标记放在桌面上")
        print("    进入 freedrive，拖拽机械臂")
        print("    使闭合指尖的几何中心触碰标记中心 → 按 Enter 记录")
        input("\n  按 Enter 进入 freedrive ...")

        robot.set_freedrive(True, timeout=99999)
        input("  指尖中心对准标记中心后按 Enter 退出 ...")
        robot.set_freedrive(False)
        time.sleep(0.3)

        flange_pose = robot.getl()
        flange_xyz = np.array(flange_pose[:3])

        # 应用 TCP 偏移 → 得到指尖中心在 base 系的位置
        flange_rot = R.from_rotvec(flange_pose[3:])
        fingertip_offset_base = flange_rot.apply(np.array(tcp_offset))
        touch_xyz = flange_xyz + fingertip_offset_base

        print(f"\n  Flange 位姿 (base系):     {fmt_arr(flange_xyz)}")
        if np.any(tcp_offset):
            print(f"  + TCP 偏移 (base系):      {fmt_arr(fingertip_offset_base)}")
            print(f"  = 指尖中心估计位置:       {fmt_arr(touch_xyz)}")
        else:
            print(f"  (TCP 偏移为 0，直接用 flange 位置)")

        # ─── Step 2: 相机拍摄 + 检测 ───
        print(f"\n  [Step 2] 相机拍摄标记 ID={target_id}")
        print("    移动机械臂使相机能看到 ArUco 标记 (~30-50cm 距离)")
        gripper.set_pos(1000)   # 张开夹爪避免遮挡
        time.sleep(1)
        input("  按 Enter 进入 freedrive 调整相机位置 ...")

        robot.set_freedrive(True, timeout=99999)
        input("  相机对准标记后按 Enter ...")
        robot.set_freedrive(False)
        time.sleep(0.3)

        # 拍照检测
        print("  拍照中...")
        try:
            rgb, depth = capture_images()
            print(f"  RGB: {rgb.shape}, Depth: {depth.shape}")
        except Exception as e:
            print(f"  ❌ 拍照失败: {e}")
            print("    请确认 D405 相机已启动 (roslaunch realsense2_camera rs_camera_d405.launch)")
            continue

        ok, marker_xyz_cam, _, corners, detected_ids = detect_aruco_marker(rgb, target_id)
        if not ok:
            if detected_ids:
                print(f"  ❌ 未检测到标记 ID={target_id}，但检测到了: {detected_ids}")
                print(f"    → 请使用 ID={target_id} 的标记，或调整 --rounds")
            else:
                print(f"  ❌ 未检测到任何 ArUco 标记")
            print("    请确认: 标记大小≥5cm, 光照充足, 相机距离 30-50cm")
            # 保存图像供检查
            debug_path = os.path.join(SCRIPT_DIR, f'../config/_debug_aruco_r{r}.jpg')
            os.makedirs(os.path.dirname(debug_path), exist_ok=True)
            cv2.imwrite(debug_path, rgb)
            print(f"    已保存调试图像: {debug_path}")
            continue

        print(f"\n  标记在相机系: {fmt_arr(marker_xyz_cam)}")

        # ─── Step 3: 坐标变换 ───
        # 相机系 → 末端系 (手眼标定)
        marker_quat_cam = np.array([0, 0, 0, 1])  # 标记平放，无旋转
        marker_ee_xyz, marker_ee_quat = transform_target_to_ee(
            EE_CAM_XYZ, EE_CAM_Q, marker_xyz_cam, marker_quat_cam
        )
        # 末端系 → 基座系
        marker_base_xyz, _ = transform_ee_to_base(robot, marker_ee_xyz, marker_ee_quat)

        print(f"  通过手眼标定计算 (base系): {fmt_arr(marker_base_xyz)}")

        # ─── Step 4: 对比 ───
        error = np.linalg.norm(touch_xyz - marker_base_xyz) * 1000  # mm
        error_xy = np.linalg.norm(touch_xyz[:2] - marker_base_xyz[:2]) * 1000
        error_z = abs(touch_xyz[2] - marker_base_xyz[2]) * 1000

        print(f"\n  {'─' * 40}")
        print(f"  误差分析:")
        print(f"    触碰位置:    {fmt_arr(touch_xyz)}")
        print(f"    相机计算:    {fmt_arr(marker_base_xyz)}")
        print(f"    ΔXY: {error_xy:.1f} mm  ← 反映手眼标定精度")
        print(f"    ΔZ:  {error_z:.1f} mm  ← 含 TCP 偏移估计误差")
        print(f"    总误差: {error:.1f} mm")
        print(f"  {'─' * 40}")

        # 以 XY 误差为主（手眼标定决定性指标），Z 受 TCP 偏移影响
        if error_xy < 10:
            print(f"  ✅ ΔXY < 1cm，手眼标定精度良好")
        elif error_xy < 30:
            print(f"  ⚠️ ΔXY 1-3cm，标定可接受但建议重新标定")
        else:
            print(f"  ❌ ΔXY > 3cm，建议重新标定")

        result = {
            "round": r + 1,
            "timestamp": datetime.now().isoformat(),
            "flange_xyz": flange_xyz.tolist(),
            "flange_rotvec": flange_pose[3:],
            "tcp_offset": list(tcp_offset),
            "touch_xyz": touch_xyz.tolist(),
            "marker_cam_xyz": marker_xyz_cam.tolist(),
            "marker_base_xyz": marker_base_xyz.tolist(),
            "error_mm": round(error, 2),
            "error_xy_mm": round(error_xy, 2),
            "error_z_mm": round(error_z, 2),
        }
        results.append(result)

        # 回到 home
        robot.movej(HOME_J, acc=0.5, vel=0.5)

    # ═══ 保存结果 ═══════════════════════════════════════
    print(f"\n{'=' * 60}")

    if results:
        os.makedirs(os.path.dirname(RESULT_FILE), exist_ok=True)
        existing = []
        if os.path.exists(RESULT_FILE):
            with open(RESULT_FILE) as f:
                existing = json.load(f)
        all_results = existing + results
        with open(RESULT_FILE, 'w') as f:
            json.dump(all_results, f, indent=2, ensure_ascii=False)
        print(f"  结果已保存: {RESULT_FILE}")

        errors = [r['error_mm'] for r in results]
        print(f"\n  汇总: {len(results)} 点")
        print(f"    平均误差: {np.mean(errors):.1f} mm")
        print(f"    最大误差: {np.max(errors):.1f} mm")
        print(f"    最小误差: {np.min(errors):.1f} mm")

    gripper.ser.close()
    robot.close()
    print("\n✅ 验证完成")


if __name__ == "__main__":
    main()
