#!/home/yuan/robot_ssh/.conda/envs/global_3.10/bin/python3
# -*- coding: utf-8 -*-
"""
手眼标定脚本 (eye-on-hand, 自包含, 不需要 MoveIt / aruco_ros / rqt)

原理: AX = XB
  A: 机械臂末端 (flange) 在两帧之间的运动
  B: 相机在两帧之间的运动 (通过 ArUco 标记检测)
  X: 相机在 flange 坐标系下的位姿 ← 这就是我们要的 ee_cam_xyz / ee_cam_q

流程:
  1. 固定 ArUco 标记在桌面上
  2. 移动机械臂到多个不同位姿 (每次都能看到标记)
  3. 每个位姿记录: flange 位姿 (robot.getl) + 标记在相机系位姿 (ArUco 检测)
  4. 用 OpenCV calibrateHandEye 求解
  5. 保存结果到 config/hand_eye_calibration.json

用法:
  python3 calibrate_hand_eye.py                    # 交互采样
  python3 calibrate_hand_eye.py --samples 15       # 指定采样数 (默认 12)
  python3 calibrate_hand_eye.py --marker-size 0.05 # 标记边长 (m)

坐标系说明:
  easy_handeye 标定时用的 robot_effector_frame = ur_arm_wrist_3_link
  UR 控制器的 flange 在姿态上等同于 wrist_3_link (没有 URDF 的额外旋转)
  robot.getl() 返回值与 wrist_3 姿态一致, 可以直接用作标定输入
"""

import sys
import os
import time
import json
import argparse
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

# ── ROS imports ──
import rospy
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo

# ── 参数 ─────────────────────────────────────────────────
ROBOT_IP = "192.168.131.3"
HOME_J = [1.60624647, -2.69281799, 2.24398565, -2.31487161, -1.60637647, 0.00013183]

# ArUco
ARUCO_DICT = cv2.aruco.DICT_4X4_50
ARUCO_MARKER_ID = 0   # 标定用的标记 ID

# 相机
CAMERA_INFO_TOPIC = "/d405/color/camera_info"
COLOR_TOPIC = "/d405/color/image_raw"
DEPTH_TOPIC = "/d405/depth/image_rect_raw"

# 保存路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_FILE = os.path.join(SCRIPT_DIR, '../config/hand_eye_calibration_new.json')

# ── 全局 ──
_camera_matrix = None
_dist_coeffs = None


# ═══════════════════════════════════════════════════════
#  相机
# ═══════════════════════════════════════════════════════

def get_camera_intrinsics():
    global _camera_matrix, _dist_coeffs
    if _camera_matrix is not None:
        return _camera_matrix, _dist_coeffs
    try:
        msg = rospy.wait_for_message(CAMERA_INFO_TOPIC, CameraInfo, timeout=3.0)
        _camera_matrix = np.array(msg.K).reshape(3, 3)
        _dist_coeffs = np.array(msg.D)
    except Exception:
        _camera_matrix = np.array([[643.0, 0, 643.0], [0, 643.0, 363.0], [0, 0, 1]])
        _dist_coeffs = np.zeros(5)
    return _camera_matrix, _dist_coeffs


def rosimg_to_numpy(msg):
    """ROS Image → numpy (不依赖 cv_bridge, 避免 conda libffi 冲突)"""
    encoding_map = {
        'bgr8': (np.uint8, 3), 'rgb8': (np.uint8, 3),
        'bgra8': (np.uint8, 4), 'rgba8': (np.uint8, 4),
        'mono8': (np.uint8, 1), '16UC1': (np.uint16, 1), '32FC1': (np.float32, 1),
    }
    dtype, channels = encoding_map.get(msg.encoding, (np.uint8, 3))
    arr = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
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
    """直接从 ROS topic 捕获图像"""
    color_msg = rospy.wait_for_message(COLOR_TOPIC, ImageMsg, timeout=timeout)
    depth_msg = rospy.wait_for_message(DEPTH_TOPIC, ImageMsg, timeout=timeout)
    if color_msg is None or depth_msg is None:
        raise RuntimeError("未收到相机图像")
    return rosimg_to_numpy(color_msg), rosimg_to_numpy(depth_msg)


# ═══════════════════════════════════════════════════════
#  ArUco 检测
# ═══════════════════════════════════════════════════════

def detect_aruco_marker(rgb_image, target_id, marker_size):
    """检测 ArUco 标记, 返回相机系下的位姿"""
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    corners, ids, _ = detector.detectMarkers(rgb_image)
    detected_ids = ids.flatten().tolist() if ids is not None else []

    if ids is None or target_id not in detected_ids:
        return False, None, None, None, detected_ids

    idx = list(detected_ids).index(target_id)
    corner = corners[idx][0]

    cam_mat, dist = get_camera_intrinsics()
    half = marker_size / 2
    obj_points = np.array([
        [-half, -half, 0], [half, -half, 0],
        [half,  half, 0], [-half,  half, 0],
    ], dtype=np.float32)

    success, rvec, tvec = cv2.solvePnP(obj_points, corner, cam_mat, dist)
    if not success:
        return False, None, None, None, detected_ids

    marker_xyz = tvec.flatten()
    marker_quat = R.from_rotvec(rvec.flatten()).as_quat()
    return True, marker_xyz, marker_quat, corner, detected_ids


# ═══════════════════════════════════════════════════════
#  采样
# ═══════════════════════════════════════════════════════

def take_sample(robot, marker_size, target_id):
    """
    一次采样:
      1. freedrive 移动机械臂到新位姿
      2. 拍照 + ArUco 检测
      3. 记录 flange 位姿 和 marker 在相机系的位姿
    """
    input("  按 Enter 进入 freedrive, 移动机械臂后按 Enter 退出...")
    robot.set_freedrive(True, timeout=99999)
    input("  到位后按 Enter 拍照...")
    robot.set_freedrive(False)
    time.sleep(0.5)

    # 记录 flange 位姿
    flange_pose = robot.getl()
    flange_xyz = np.array(flange_pose[:3])
    flange_rotvec = np.array(flange_pose[3:])

    # 拍照 + 检测
    rgb, _ = capture_images()
    ok, marker_xyz, marker_quat, _, detected_ids = detect_aruco_marker(rgb, target_id, marker_size)

    if not ok:
        if detected_ids:
            print(f"  ❌ 未检测到标记 ID={target_id}, 检测到了: {detected_ids}")
        else:
            print(f"  ❌ 未检测到任何 ArUco 标记")
        return None

    print(f"  ✅ flange:   [{flange_xyz[0]:.4f}, {flange_xyz[1]:.4f}, {flange_xyz[2]:.4f}]")
    print(f"     marker:   [{marker_xyz[0]:.4f}, {marker_xyz[1]:.4f}, {marker_xyz[2]:.4f}]")

    return {
        'flange_xyz': flange_xyz.tolist(),
        'flange_rotvec': flange_rotvec.tolist(),
        'marker_cam_xyz': marker_xyz.tolist(),
        'marker_cam_quat': marker_quat.tolist(),
    }


def collect_samples(robot, args):
    """交互式采集多组样本"""
    samples = []
    target_id = ARUCO_MARKER_ID

    print(f"\n{'=' * 60}")
    print(f"  采样 ({len(samples) + 1}/{args.samples})")
    print(f"{'=' * 60}")

    # 判断是否使用 freedrive
    if not args.freehand:
        print("\n  ⚠️  手动模式: 每次需要手动移动机械臂到新位姿")
        print("     确保 ArUco 标记始终在相机视野中 (~30-50cm)")
        print("     位姿要覆盖不同角度和距离 (平移+旋转)")

    while len(samples) < args.samples:
        print(f"\n{'─' * 60}")
        print(f"  样本 {len(samples) + 1}/{args.samples}")
        print(f"{'─' * 60}")

        sample = take_sample(robot, args.marker_size, target_id)

        if sample is None:
            retry = input("  重试? (y/n): ").strip().lower()
            if retry == 'n':
                if samples:
                    print(f"  已采集 {len(samples)} 个样本, 提前结束")
                    break
                else:
                    continue
            continue

        samples.append(sample)
        print(f"  已采集: {len(samples)}/{args.samples}")

        if len(samples) >= args.samples:
            break

        # 建议下一个位姿
        if len(samples) < args.samples:
            print(f"\n  💡 建议: 改变相机角度/距离")
            print(f"     当前 flange_xyz range:")
            x_vals = [s['flange_xyz'][0] for s in samples]
            y_vals = [s['flange_xyz'][1] for s in samples]
            z_vals = [s['flange_xyz'][2] for s in samples]
            print(f"     X: [{min(x_vals):.3f}, {max(x_vals):.3f}] "
                  f"Y: [{min(y_vals):.3f}, {max(y_vals):.3f}] "
                  f"Z: [{min(z_vals):.3f}, {max(z_vals):.3f}]")

    return samples


# ═══════════════════════════════════════════════════════
#  手眼标定求解
# ═══════════════════════════════════════════════════════

def solve_hand_eye(samples):
    """
    用 OpenCV calibrateHandEye 求解 AX = XB

    A_i = inv(T_flange_i) * T_flange_{i+1}   (末端在基座系下的运动)
    B_i = T_marker_{i+1} * inv(T_marker_i)   (标记在相机系下的运动)
    X   = T_camera_to_flange                 (相机在末端系下的位姿)

    返回: ee_cam_xyz, ee_cam_q
    """
    if len(samples) < 3:
        raise RuntimeError(f"至少需要 3 个样本, 当前 {len(samples)}")

    # 构建位姿矩阵列表
    T_flange_list = []
    T_marker_list = []

    for s in samples:
        # flange 在 base 系的位姿
        R_f = R.from_rotvec(s['flange_rotvec']).as_matrix()
        t_f = np.array(s['flange_xyz']).reshape(3, 1)
        T_f = np.vstack([np.hstack([R_f, t_f]), [[0, 0, 0, 1]]])
        T_flange_list.append(T_f)

        # marker 在 camera 系的位姿
        R_m = R.from_quat(s['marker_cam_quat']).as_matrix()
        t_m = np.array(s['marker_cam_xyz']).reshape(3, 1)
        T_m = np.vstack([np.hstack([R_m, t_m]), [[0, 0, 0, 1]]])
        T_marker_list.append(T_m)

    # 计算相对运动 A 和 B
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for i in range(len(T_flange_list) - 1):
        # A: flange运动 (从 i 到 i+1, 在 flange 系中)
        T_A = np.linalg.inv(T_flange_list[i]) @ T_flange_list[i + 1]
        R_gripper2base.append(T_A[:3, :3])
        t_gripper2base.append(T_A[:3, 3])

        # B: marker运动 (从 i 到 i+1, 在 camera 系中)
        T_B = T_marker_list[i + 1] @ np.linalg.inv(T_marker_list[i])
        R_target2cam.append(T_B[:3, :3])
        t_target2cam.append(T_B[:3, 3])

    print(f"\n  求解中... (基于 {len(R_gripper2base)} 个运动对)")

    # 用 OpenCV 求解
    # calibrateHandEye 返回: R_cam2gripper, t_cam2gripper
    try:
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
    except Exception as e:
        print(f"  ❌ OpenCV 求解失败: {e}")
        print(f"  尝试用其他方法...")
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_PARK
        )

    ee_cam_xyz = t_cam2gripper.flatten()
    ee_cam_q = R.from_matrix(R_cam2gripper).as_quat()

    return ee_cam_xyz, ee_cam_q, R_gripper2base, R_target2cam


# ═══════════════════════════════════════════════════════
#  验证: 用标定结果计算标记在 base 系的位置, 与 freedrive 触碰对比
# ═══════════════════════════════════════════════════════

def verify_calibration(robot, ee_cam_xyz, ee_cam_q, marker_size, tcp_offset):
    """单点验证手眼标定: freedrive 触碰标记 → 对比相机计算结果"""
    print(f"\n{'=' * 60}")
    print(f"  标定验证")
    print(f"{'=' * 60}")
    print("  1. 用指尖触碰 ArUco 标记中心")
    print("  2. 拍照, 用新标定计算标记位置")
    print("  3. 对比误差")

    # Step 1: 触碰
    print("\n  [Step 1] 触碰标记")
    input("  按 Enter 进入 freedrive, 指尖触碰标记中心后按 Enter...")
    robot.set_freedrive(True, timeout=99999)
    input("  触碰到位, 按 Enter 记录...")
    robot.set_freedrive(False)
    time.sleep(0.3)

    flange_pose = robot.getl()
    flange_xyz = np.array(flange_pose[:3])
    flange_rot = R.from_rotvec(flange_pose[3:])

    # 指尖在 base 系的位置
    fingertip_offset_base = flange_rot.apply(np.array(tcp_offset))
    touch_xyz = flange_xyz + fingertip_offset_base
    print(f"  指尖 (base系): [{touch_xyz[0]:.4f}, {touch_xyz[1]:.4f}, {touch_xyz[2]:.4f}]")

    # Step 2: 拍照
    print("\n  [Step 2] 相机拍摄")
    input("  按 Enter 进入 freedrive, 调整相机位置后按 Enter...")
    robot.set_freedrive(True, timeout=99999)
    input("  相机对准标记后按 Enter...")
    robot.set_freedrive(False)
    time.sleep(0.3)

    rgb, _ = capture_images()
    ok, marker_xyz_cam, marker_quat_cam, _, detected_ids = detect_aruco_marker(
        rgb, ARUCO_MARKER_ID, marker_size)

    if not ok:
        print(f"  ❌ 检测失败: {detected_ids}")
        return None

    # Step 3: 变换
    R_cam_ee = R.from_quat(ee_cam_q).as_matrix()
    t_cam_ee = np.array(ee_cam_xyz)
    marker_ee = R_cam_ee @ marker_xyz_cam + t_cam_ee

    ee_pose = robot.getl()
    R_base_ee = R.from_rotvec(ee_pose[3:]).as_matrix()
    t_base_ee = np.array(ee_pose[:3])
    marker_base = R_base_ee @ marker_ee + t_base_ee

    # Step 4: 对比
    error = np.linalg.norm(touch_xyz - marker_base) * 1000
    error_xy = np.linalg.norm(touch_xyz[:2] - marker_base[:2]) * 1000
    error_z = abs(touch_xyz[2] - marker_base[2]) * 1000

    print(f"\n  触碰:  [{touch_xyz[0]:.4f}, {touch_xyz[1]:.4f}, {touch_xyz[2]:.4f}]")
    print(f"  相机:  [{marker_base[0]:.4f}, {marker_base[1]:.4f}, {marker_base[2]:.4f}]")
    print(f"  ΔXY = {error_xy:.1f} mm,  ΔZ = {error_z:.1f} mm,  总 = {error:.1f} mm")

    return {'error_mm': error, 'error_xy_mm': error_xy, 'error_z_mm': error_z}


# ═══════════════════════════════════════════════════════
#  主流程
# ═══════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="手眼标定 (eye-on-hand)")
    parser.add_argument("--samples", type=int, default=12,
                        help="采样数 (默认 12, 建议 10-20)")
    parser.add_argument("--marker-size", type=float, default=0.05,
                        help="ArUco 标记边长 (m), 默认 0.05")
    parser.add_argument("--marker-id", type=int, default=ARUCO_MARKER_ID,
                        help=f"ArUco 标记 ID, 默认 {ARUCO_MARKER_ID}")
    parser.add_argument("--freehand", action="store_true",
                        help="手动移动机械臂 (不用 freedrive)")
    parser.add_argument("--skip-verify", action="store_true",
                        help="跳过标定后验证")
    parser.add_argument("--tcp-offset", type=float, nargs=3,
                        default=[0.0, 0.0, 0.13],
                        help="闭合指尖在 flange 系下的偏移 (m), 默认 [0,0,0.13]")
    parser.add_argument("--load", type=str,
                        help="从已有样本文件求解 (跳过采样)")
    args = parser.parse_args()

    tcp_offset = np.array(args.tcp_offset)

    # ═══ 初始化 ═══════════════════════════════════════
    print("=" * 60)
    print("  手眼标定 (eye-on-hand)")
    print("=" * 60)

    rospy.init_node("calibrate_hand_eye", anonymous=True, disable_signals=True)
    get_camera_intrinsics()
    print(f"✅ 相机内参: fx={_camera_matrix[0,0]:.1f}, fy={_camera_matrix[1,1]:.1f}")

    if args.load:
        # ── 从文件加载已有样本 ──
        print(f"\n从文件加载样本: {args.load}")
        with open(args.load) as f:
            samples = json.load(f)
        print(f"✅ 已加载 {len(samples)} 个样本")

    else:
        # ── 连接 + 采样 ──
        print(f"\n连接 UR 机械臂 {ROBOT_IP} ...")
        robot = urx.Robot(ROBOT_IP)
        data = robot.secmon.get_all_data()
        mode = data.get('RobotModeData', {}).get('robotMode', -1)
        if mode != 7:
            print(f"❌ 机械臂未就绪 (mode={mode})")
            robot.close()
            sys.exit(1)
        print("✅ 机械臂就绪")

        print("\n→ 回 home...")
        robot.movej(HOME_J, acc=0.5, vel=0.5)
        time.sleep(0.5)

        print(f"\n{'=' * 60}")
        print(f"  开始采样")
        print(f"  标记 ID: {args.marker_id}, 边长: {args.marker_size}m")
        print(f"  目标: {args.samples} 个样本")
        print(f"{'=' * 60}")
        print("\n  📌 确保 ArUco 标记固定在桌面上不动")
        print("  📌 每次移动机械臂到不同的位姿")
        print("  📌 标记始终在相机视野中 (~30-50cm)")
        print("  📌 覆盖不同角度: 正对 / 左倾 / 右倾 / 远 / 近")

        samples = collect_samples(robot, args)

        if len(samples) < 3:
            print(f"❌ 样本不足 ({len(samples)}), 至少需要 3 个")
            robot.close()
            sys.exit(1)

        # 保存原始样本 (可重复使用)
        sample_file = os.path.join(SCRIPT_DIR,
                                   f'../config/hand_eye_samples_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
        os.makedirs(os.path.dirname(sample_file), exist_ok=True)
        with open(sample_file, 'w') as f:
            json.dump(samples, f, indent=2)
        print(f"\n✅ 样本已保存: {sample_file}")

    # ═══ 求解 ═══════════════════════════════════════
    print(f"\n{'=' * 60}")
    print(f"  求解 AX = XB")
    print(f"{'=' * 60}")
    ee_cam_xyz, ee_cam_q, R_gripper, R_target = solve_hand_eye(samples)

    print(f"\n  ✅ 标定结果:")
    print(f"  ee_cam_xyz = [{ee_cam_xyz[0]:.6f}, {ee_cam_xyz[1]:.6f}, {ee_cam_xyz[2]:.6f}]")
    print(f"  ee_cam_q   = [{ee_cam_q[0]:.6f}, {ee_cam_q[1]:.6f}, "
          f"{ee_cam_q[2]:.6f}, {ee_cam_q[3]:.6f}]")

    # 保存
    result = {
        'timestamp': datetime.now().isoformat(),
        'num_samples': len(samples),
        'marker_size': args.marker_size,
        'marker_id': args.marker_id,
        'ee_cam_xyz': ee_cam_xyz.tolist(),
        'ee_cam_q': ee_cam_q.tolist(),
        'tcp_offset': args.tcp_offset,
    }
    os.makedirs(os.path.dirname(RESULT_FILE), exist_ok=True)
    with open(RESULT_FILE, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"\n  结果已保存: {RESULT_FILE}")

    # ═══ 验证 ═══════════════════════════════════════
    if not args.skip_verify and not args.load:
        verify = verify_calibration(robot, ee_cam_xyz, ee_cam_q,
                                    args.marker_size, tcp_offset)
        if verify:
            result['verification'] = verify
            with open(RESULT_FILE, 'w') as f:
                json.dump(result, f, indent=2)

    if not args.load:
        robot.movej(HOME_J, acc=0.5, vel=0.5)
        robot.close()

    print(f"\n{'=' * 60}")
    print(f"  下一步:")
    print(f"  将以下两行复制到 verify_hand_eye.py 的 EE_CAM 定义处:")
    print(f"{'=' * 60}")
    print(f"ee_cam_xyz = np.array([{ee_cam_xyz[0]:.6f}, {ee_cam_xyz[1]:.6f}, {ee_cam_xyz[2]:.6f}])")
    print(f"ee_cam_q   = np.array([{ee_cam_q[0]:.6f}, {ee_cam_q[1]:.6f}, "
          f"{ee_cam_q[2]:.6f}, {ee_cam_q[3]:.6f}])")
    print(f"\n✅ 标定完成")


if __name__ == "__main__":
    main()
