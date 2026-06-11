#!/usr/bin/env python3
"""
upright_cup_pose_node.py

Hand-eye 비전 노드 (cup-stack-integration v1.1).

이 노드는 hand-eye 카메라로 본 upright(똑바로 선) 컵들을 검출하고, **카메라 광학
좌표 → base_link 변환까지 직접 수행**해서 모든 컵을 `/hand_eye/boxes`
(visualization_msgs/MarkerArray, base_link frame) 로 발행한다. 즉 fake_hand_eye_node
의 실(real) 대체물이다. pick_node 는 이 토픽을 그대로 받아(좌표변환 없이) EE 최근접
컵을 골라 pyramid API 를 호출한다.

좌표 변환 (dsr_practice/stand_fallen_cup.py 검증식 재사용):
    T_base_ee  = get_ee_matrix(robot)          # link_6 global transform (MoveItPy FK)
    T_ee_cam   = gripper2cam (npy, mm→m)
    T_base_cam = T_base_ee @ T_ee_cam
    p_base     = (T_base_cam @ [p_cam, 1])[:3] - base_offset

Output:
  /hand_eye/boxes : visualization_msgs/MarkerArray (base_link frame)
      - 매 발행마다 DELETEALL 1개로 스냅샷 초기화 후, 컵마다 2개:
        ns="box_top"    id=i  pose.position=(x,y,z)  ← base_link 좌표
        ns="box_labels" id=i  text="#i_c=<color>_upright-cup"
  /upright_cup/debug_image : Image (검출/선택 시각화)

전제:
  - dsr_bringup2_moveit.launch.py 가 떠 있어야 MoveItPy FK 가능.
  - hand-eye 캘리브 파일(T_gripper2camera.npy) 이 calib_file 경로에 있어야 함.
"""

import math
import time

import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

from ultralytics import YOLO


# cv_bridge 대체 (numpy 1.x↔2.x ABI 충돌 회피).
def imgmsg_to_cv2(msg, desired_encoding="passthrough"):
    """sensor_msgs/Image → cv2/numpy. desired_encoding은 bgr8 또는 passthrough."""
    h, w = msg.height, msg.width
    enc = msg.encoding

    if enc == "16UC1":
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
    elif enc == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
    elif enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
    elif enc in ("bgr8", "rgb8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif enc in ("bgra8", "rgba8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
    else:
        raise ValueError(f"Unsupported encoding: {enc}")

    if desired_encoding == "passthrough" or desired_encoding == enc:
        return arr.copy()

    if desired_encoding == "bgr8":
        if enc == "rgb8":
            return arr[:, :, ::-1].copy()
        if enc == "bgra8":
            return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        if enc == "rgba8":
            return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        if enc == "mono8":
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    raise ValueError(f"Cannot convert {enc} → {desired_encoding}")


def cv2_to_imgmsg(image, encoding="bgr8"):
    """cv2/numpy → sensor_msgs/Image. bgr8/rgb8/mono8 지원."""
    msg = Image()
    h, w = image.shape[:2]
    msg.height = h
    msg.width = w
    msg.encoding = encoding
    msg.is_bigendian = 0
    if encoding in ("bgr8", "rgb8"):
        msg.step = w * 3
    elif encoding == "mono8":
        msg.step = w
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")
    msg.data = image.tobytes()
    return msg


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ["true", "1", "yes", "y"]
    return bool(value)


# stand_fallen_cup.py 의 get_ee_matrix 와 동일 (link_6 global transform).
EE_LINK = "link_6"


def get_ee_matrix(moveit_robot):
    """현재 link_6(EE) 의 base_link 기준 4x4 변환 (planning scene read-only)."""
    psm = moveit_robot.get_planning_scene_monitor()
    with psm.read_only() as scene:
        T = scene.current_state.get_global_link_transform(EE_LINK)
    return np.asarray(T, dtype=float)


# HSV 기반 단순 색 분류. mask 영역 평균 BGR → 알려진 컵 색 토큰.
# plan_executor/pick_node 의 _KNOWN_COLORS 와 어휘 일치.
def classify_color_bgr(mean_bgr):
    b, g, r = [float(c) for c in mean_bgr]
    px = np.uint8([[[b, g, r]]])
    hsv = cv2.cvtColor(px, cv2.COLOR_BGR2HSV)[0, 0]
    h, s, v = int(hsv[0]), int(hsv[1]), int(hsv[2])
    if v < 50:
        return "black"
    if s < 40:
        return "white" if v > 180 else "gray"
    # OpenCV Hue: 0-179
    if h < 10 or h >= 170:
        return "red"
    if h < 25:
        return "orange"
    if h < 35:
        return "yellow"
    if h < 85:
        return "green"
    if h < 130:
        return "blue"
    if h < 160:
        return "purple"
    return "red"


class CupTracker:
    """base_link 공간 컵 트래커 — 프레임 간 매칭 후 EMA 평활 + outlier 제거.

    정지 컵은 base_link 에서 좌표가 고정이므로(카메라가 움직여도), base 공간에서
    평활하면 카메라 모션과 무관하게 per-frame 튐(볼트구멍 오선택 등)을 걸러낸다.
    단발 outlier 는 무시(평활값 유지), 같은 방향으로 연속되면 컵이 실제로 옮겨진
    것으로 보고 재획득한다. 트랙 id 는 안정적으로 유지해 마커 id 로도 쓴다.
    """

    def __init__(self, match_dist, alpha, outlier_dist, reacquire_frames,
                 timeout_sec, min_hits):
        self.match_dist = float(match_dist)
        self.alpha = float(alpha)
        self.outlier_dist = float(outlier_dist)
        self.reacquire_frames = int(reacquire_frames)
        self.timeout_sec = float(timeout_sec)
        self.min_hits = int(min_hits)
        self.tracks = []          # 각 트랙: dict(id,xyz,color,last_seen,hits,outliers)
        self._next_id = 0

    def update(self, cups, now_sec):
        """cups: [{"xy_base":(x,y),"z_base":z,"color":str,...}] → 평활된 cups 반환."""
        meas = [np.array([c["xy_base"][0], c["xy_base"][1], c["z_base"]], float)
                for c in cups]

        # ── 그리디 최근접 매칭 (xy 거리 ≤ match_dist) ──
        pairs = []
        for mi, p in enumerate(meas):
            for ti, tr in enumerate(self.tracks):
                d = math.hypot(p[0] - tr["xyz"][0], p[1] - tr["xyz"][1])
                if d <= self.match_dist:
                    pairs.append((d, mi, ti))
        pairs.sort(key=lambda x: x[0])
        m_used, t_used = set(), set()
        for d, mi, ti in pairs:
            if mi in m_used or ti in t_used:
                continue
            m_used.add(mi); t_used.add(ti)
            self._update_track(self.tracks[ti], meas[mi], cups[mi], now_sec)

        # ── 매칭 안 된 측정 → 새 트랙 ──
        for mi, p in enumerate(meas):
            if mi in m_used:
                continue
            self.tracks.append({
                "id": self._next_id, "xyz": p.copy(),
                "color": cups[mi]["color"], "last_seen": now_sec,
                "hits": 1, "outliers": 0,
            })
            self._next_id += 1

        # ── 오래된 트랙 폐기 ──
        self.tracks = [t for t in self.tracks
                       if now_sec - t["last_seen"] <= self.timeout_sec]

        # ── 이번 프레임에 관측되고 충분히 확인된 트랙만 발행 ──
        out = []
        for t in self.tracks:
            if t["last_seen"] == now_sec and t["hits"] >= self.min_hits:
                out.append({
                    "xy_base": (float(t["xyz"][0]), float(t["xyz"][1])),
                    "z_base": float(t["xyz"][2]),
                    "color": t["color"], "id": int(t["id"]),
                })
        return out

    def _update_track(self, tr, p, cup, now_sec):
        resid = math.hypot(p[0] - tr["xyz"][0], p[1] - tr["xyz"][1])
        if resid > self.outlier_dist:
            # 단발 outlier 는 무시(평활값 유지). 연속되면 컵이 실제 이동 → 재획득.
            tr["outliers"] += 1
            if tr["outliers"] >= self.reacquire_frames:
                tr["xyz"] = p.copy()
                tr["outliers"] = 0
        else:
            a = self.alpha
            tr["xyz"] = (1.0 - a) * tr["xyz"] + a * p
            tr["outliers"] = 0
        tr["color"] = cup["color"]
        tr["last_seen"] = now_sec
        tr["hits"] += 1


class UprightCupPoseNode(Node):
    """hand-eye 카메라 → base_link 변환까지 떠안고 /hand_eye/boxes 를 내는 비전 노드.

    fake_hand_eye_node 와 동일한 토픽/메시지 형식(base_link MarkerArray, box_top +
    box_labels)을 발행하므로 pick_node 입장에선 fake/real 구분 없이 동일하게 동작.
    """

    def __init__(self):
        super().__init__("upright_cup_pose_node")

        # ── Parameters ───────────────────────────────────────
        self.declare_parameter("weights_path", "")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )

        self.declare_parameter("boxes_topic", "/hand_eye/boxes")
        self.declare_parameter("debug_image_topic", "/upright_cup/debug_image")

        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("half", False)

        self.declare_parameter("target_class_name", "upright-cup")
        self.declare_parameter("min_mask_area", 300.0)
        # 중복 검출 제거: pick point 가 이 거리(px) 안인 같은 클래스 검출은
        # conf 높은 것만 남긴다. 0 이하면 비활성. (YOLO NMS 가 못 거른 겹침 정리)
        self.declare_parameter("dedup_min_dist_px", 25.0)

        # ── 시간 평활/트래킹 (base_link 공간) ────────────────
        # 검출 컵을 프레임 간 추적해 EMA 평활 + outlier(볼트구멍 오선택 등) 제거.
        # hand-eye 카메라가 움직여도 정지 컵은 base_link 에서 고정이라 base 공간에서
        # 평활하면 카메라 모션과 무관하게 튐을 걸러낸다.
        self.declare_parameter("enable_temporal_smoothing", True)
        # match_dist 는 컵 간격(≈0.10m)보다 작고 outlier_dist 보다는 커야 한다
        # (outlier 측정이 트랙에 붙어서 게이트로 걸러지도록).
        self.declare_parameter("track_match_dist", 0.08)     # 같은 컵 매칭 거리(m)
        self.declare_parameter("smoothing_alpha", 0.4)        # EMA 계수(클수록 빠름)
        self.declare_parameter("track_outlier_dist", 0.04)    # 이 이상 튀면 outlier(m)
        self.declare_parameter("track_reacquire_frames", 4)   # 연속 outlier 시 재획득
        self.declare_parameter("track_timeout_sec", 0.5)      # 미검출 트랙 폐기(s)
        self.declare_parameter("track_min_hits", 2)           # 발행 전 최소 관측수

        # ── pick point 산출 방식 ────────────────────────────
        # 똑바로 선 컵을 위에서 보면 윗면 원(rim)이 보이는데, seg mask 에 옆면이
        # 같이 잡혀 길쭉해지면 moments 무게중심이 원 중심에서 벗어난다. 그래서
        # mask 에서 "원 부분"만 다시 잡아 그 중심을 pick point 로 쓴다.
        #   top_hole  : 윗면 도넛 홀(어두운 중앙 구멍) 중심 — 컵 입구/관통홀 정밀 pick
        #   inscribed : distance transform 최댓값 위치 = 가장 큰 내접원 중심 (강건 기본)
        #   hough     : 이미지에서 HoughCircles 로 rim 원을 직접 검출
        #   centroid  : 기존 moments 무게중심 (변경 없음)
        self.declare_parameter("pick_point_method", "inscribed")
        # hough 전용 튜닝값 (반지름 비율은 contour bbox 짧은 변 기준).
        self.declare_parameter("hough_dp", 1.2)
        self.declare_parameter("hough_param1", 100.0)
        self.declare_parameter("hough_param2", 25.0)
        self.declare_parameter("hough_min_radius_ratio", 0.25)
        self.declare_parameter("hough_max_radius_ratio", 0.75)
        # top_hole 튜닝값. 밝기 임계는 Otsu(조명 자동적응)를 기본으로 쓰고
        # dark_percentile 은 Otsu 가 비정상일 때의 안전 상한이다.
        # 탐색 반경 = 내접원 r×이값. 기운 컵은 입구 중심이 내접원(몸통쪽 치우침)에서
        # 멀어 작게 잡으면 입구가 후보에서 빠진다 → 넉넉히(>=2) 둬 입구를 포함시킨다.
        self.declare_parameter("top_hole_face_ratio", 2.5)
        self.declare_parameter("top_hole_min_circularity", 0.45)  # 원형도 하한(그림자 제거)
        self.declare_parameter("top_hole_dark_percentile", 35.0)  # Otsu 안전 상한(%)
        self.declare_parameter("top_hole_min_area_frac", 0.01)    # 윗면 대비 홀 최소 면적비
        self.declare_parameter("top_hole_max_area_frac", 0.7)     # 윗면 대비 홀 최대 면적비
        # 선택 점수 = 면적 × (1 − penalty·(dist/face_r)²). 0 이면 순수 최대 면적,
        # 클수록 가장자리(볼트홀/그림자) 감점 ↑. 면적 지배로 center 구멍을 고른다.
        self.declare_parameter("top_hole_centrality_penalty", 0.4)

        # ── 좌표 변환 (camera → base_link) ──────────────────
        self.declare_parameter("base_frame", "base_link")
        # 캘리브 파일. 비우면 pick_node share 의 T_gripper2camera.npy 사용.
        self.declare_parameter("calib_file", "")
        self.declare_parameter("calib_scale_mm_to_m", True)
        self.declare_parameter("base_offset_x", 0.0)
        self.declare_parameter("base_offset_y", 0.0)
        self.declare_parameter("base_offset_z", 0.080)

        # 색 분류: 비우면 mask 평균색 자동 분류. 값 지정 시 모든 컵에 고정 색.
        self.declare_parameter("cup_color", "")

        self.weights_path = str(self.get_parameter("weights_path").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)

        self.boxes_topic = str(self.get_parameter("boxes_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf = float(self.get_parameter("conf").value)
        self.iou = float(self.get_parameter("iou").value)
        self.device = str(self.get_parameter("device").value)
        self.half = as_bool(self.get_parameter("half").value)

        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.min_mask_area = float(self.get_parameter("min_mask_area").value)
        self.dedup_min_dist_px = float(
            self.get_parameter("dedup_min_dist_px").value)

        self.enable_temporal_smoothing = as_bool(
            self.get_parameter("enable_temporal_smoothing").value)
        self.tracker = CupTracker(
            match_dist=float(self.get_parameter("track_match_dist").value),
            alpha=float(self.get_parameter("smoothing_alpha").value),
            outlier_dist=float(self.get_parameter("track_outlier_dist").value),
            reacquire_frames=int(self.get_parameter("track_reacquire_frames").value),
            timeout_sec=float(self.get_parameter("track_timeout_sec").value),
            min_hits=int(self.get_parameter("track_min_hits").value),
        )

        self.pick_point_method = str(
            self.get_parameter("pick_point_method").value).strip().lower()
        if self.pick_point_method not in (
                "top_hole", "inscribed", "hough", "centroid"):
            self.get_logger().warn(
                f"unknown pick_point_method '{self.pick_point_method}', "
                f"falling back to 'inscribed'")
            self.pick_point_method = "inscribed"
        self.hough_dp = float(self.get_parameter("hough_dp").value)
        self.hough_param1 = float(self.get_parameter("hough_param1").value)
        self.hough_param2 = float(self.get_parameter("hough_param2").value)
        self.hough_min_radius_ratio = float(
            self.get_parameter("hough_min_radius_ratio").value)
        self.hough_max_radius_ratio = float(
            self.get_parameter("hough_max_radius_ratio").value)
        self.top_hole_face_ratio = float(
            self.get_parameter("top_hole_face_ratio").value)
        self.top_hole_min_circularity = float(
            self.get_parameter("top_hole_min_circularity").value)
        self.top_hole_dark_percentile = float(
            self.get_parameter("top_hole_dark_percentile").value)
        self.top_hole_min_area_frac = float(
            self.get_parameter("top_hole_min_area_frac").value)
        self.top_hole_max_area_frac = float(
            self.get_parameter("top_hole_max_area_frac").value)
        self.top_hole_centrality_penalty = float(
            self.get_parameter("top_hole_centrality_penalty").value)

        self.base_frame = str(self.get_parameter("base_frame").value)
        calib_file = str(self.get_parameter("calib_file").value)
        self.calib_scale = as_bool(
            self.get_parameter("calib_scale_mm_to_m").value)
        self.base_offset = np.array([
            float(self.get_parameter("base_offset_x").value),
            float(self.get_parameter("base_offset_y").value),
            float(self.get_parameter("base_offset_z").value),
        ], dtype=float)
        self.cup_color = str(self.get_parameter("cup_color").value).strip().lower()

        if self.weights_path == "":
            raise RuntimeError("weights_path is empty.")

        if self.device != "cpu" and not torch.cuda.is_available():
            self.get_logger().warn("CUDA is not available. Falling back to CPU.")
            self.device = "cpu"
            self.half = False
        if self.device == "cpu":
            self.half = False

        # ── 캘리브 로드 (T_ee_cam) ──────────────────────────
        if calib_file == "":
            from ament_index_python.packages import get_package_share_directory
            from pathlib import Path
            calib_file = str(
                Path(get_package_share_directory("hand_pick"))
                / "config" / "T_gripper2camera.npy"
            )
        self.gripper2cam = np.load(calib_file).astype(float)
        if self.calib_scale:
            self.gripper2cam[:3, 3] /= 1000.0  # mm → m
        self.get_logger().info(f"Hand-Eye 캘리브 로드: {calib_file}")

        # ── MoveItPy (link_6 FK) ────────────────────────────
        from moveit.planning import MoveItPy
        self.get_logger().info("MoveItPy 초기화 중…")
        self.robot = MoveItPy(node_name="upright_cup_pose_moveit_py")
        self.get_logger().info("MoveItPy 초기화 완료")

        # ── 카메라 내부 파라미터 / depth ────────────────────
        self.last_depth_m = None
        self.fx = self.fy = self.cx = self.cy = None

        self.get_logger().info(f"Loading YOLO model: {self.weights_path}")
        self.model = YOLO(self.weights_path)
        try:
            self.model.fuse()
        except Exception as e:
            self.get_logger().warn(f"model.fuse() skipped: {e}")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos)
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, qos)

        self.boxes_pub = self.create_publisher(
            MarkerArray, self.boxes_topic, 10)
        self.debug_pub = self.create_publisher(
            Image, self.debug_image_topic, 10)

        self.get_logger().info("upright_cup_pose_node started.")
        self.get_logger().info(f"  image_topic : {self.image_topic}")
        self.get_logger().info(f"  boxes_topic : {self.boxes_topic} ({self.base_frame})")
        self.get_logger().info(f"  target_class: '{self.target_class_name}'")
        self.get_logger().info(f"  model classes: {getattr(self.model, 'names', None)}")

    # ── Depth / camera info ──────────────────────────────────
    def depth_callback(self, msg: Image):
        try:
            depth = imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"depth conversion failed: {e}")
            return
        if msg.encoding == "16UC1":
            self.last_depth_m = depth.astype(np.float32) * 0.001
        elif msg.encoding == "32FC1":
            self.last_depth_m = depth.astype(np.float32)
        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def get_depth_at_pixel(self, u, v, window=7):
        if self.last_depth_m is None:
            return None
        h, w = self.last_depth_m.shape[:2]
        u = int(round(u))
        v = int(round(v))
        if u < 0 or u >= w or v < 0 or v >= h:
            return None
        r = window // 2
        x0, x1 = max(0, u - r), min(w, u + r + 1)
        y0, y1 = max(0, v - r), min(h, v + r + 1)
        patch = self.last_depth_m[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch)]
        valid = valid[valid > 0.05]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def deproject_pixel_to_3d(self, u, v, z):
        if None in (self.fx, self.fy, self.cx, self.cy):
            return None
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    # ── YOLO mask extraction ─────────────────────────────────
    def extract_detections(self, result, frame_bgr, image_h, image_w):
        detections = []
        if result.masks is None or result.masks.data is None:
            return detections
        masks = result.masks.data.detach().cpu().numpy()
        boxes = result.boxes
        confs = clss = None
        if boxes is not None:
            if boxes.conf is not None:
                confs = boxes.conf.detach().cpu().numpy()
            if boxes.cls is not None:
                clss = boxes.cls.detach().cpu().numpy()

        for i, mask in enumerate(masks):
            if mask.shape[:2] != (image_h, image_w):
                mask = cv2.resize(
                    mask, (image_w, image_h), interpolation=cv2.INTER_NEAREST)
            binary = (mask > 0.5).astype(np.uint8) * 255
            contours, _ = cv2.findContours(
                binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                continue
            contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(contour))
            if area < self.min_mask_area:
                continue
            M = cv2.moments(contour)
            if abs(M["m00"]) < 1e-6:
                continue
            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])
            centroid = np.array([cx, cy], dtype=np.float32)

            # 옆면이 같이 잡혀 길쭉해진 mask 에서 "원(rim)" 중심을 다시 잡는다.
            center, radius = self.compute_pick_point(
                frame_bgr, binary, contour, centroid)

            conf = float(confs[i]) if confs is not None and i < len(confs) else 1.0
            cls_id = int(clss[i]) if clss is not None and i < len(clss) else -1
            detections.append({
                "mask": binary,
                "contour": contour,
                "area": area,
                "center": center,        # pick point (원 중심)
                "centroid": centroid,    # 기존 무게중심 (debug 비교용)
                "pick_radius": radius,   # 검출된 원 반지름(px) 또는 None
                "conf": conf,
                "cls_id": cls_id,
                "cls_name": self._class_id_to_name(cls_id),
            })
        return self._dedup_detections(detections)

    def _dedup_detections(self, detections):
        """pick point 가 dedup_min_dist_px 안인 **같은 클래스** 검출은 conf 높은
        것만 남긴다. YOLO NMS 가 못 거른 겹친 중복 검출(같은 컵 두 번)을 정리."""
        if self.dedup_min_dist_px <= 0 or len(detections) < 2:
            return detections
        thr2 = self.dedup_min_dist_px ** 2
        kept = []
        for det in sorted(detections, key=lambda d: d["conf"], reverse=True):
            c = det["center"]
            dup = False
            for k in kept:
                if k["cls_id"] != det["cls_id"]:
                    continue
                kc = k["center"]
                if (c[0] - kc[0]) ** 2 + (c[1] - kc[1]) ** 2 <= thr2:
                    dup = True
                    break
            if not dup:
                kept.append(det)
        return kept

    # ── pick point: mask 의 "원" 중심 산출 ────────────────────
    def compute_pick_point(self, frame_bgr, binary, contour, centroid):
        """선택된 방식으로 pick point (u,v) 와 원 반지름(px, 없으면 None) 반환.

        모든 방식은 실패 시 moments 무게중심(centroid)으로 폴백한다.
        """
        method = self.pick_point_method
        if method == "centroid":
            return centroid, None
        if method == "top_hole":
            res = self._top_hole(frame_bgr, binary, centroid)
            if res is not None:
                return res
            # 홀 검출 실패 → 내접원으로 폴백
            return self._inscribed_circle(binary, centroid)
        if method == "hough":
            res = self._hough_circle(frame_bgr, contour, centroid)
            if res is not None:
                return res
            # hough 실패 → 내접원으로 폴백
            return self._inscribed_circle(binary, centroid)
        # 기본: inscribed
        return self._inscribed_circle(binary, centroid)

    def _inscribed_circle(self, binary, centroid):
        """distance transform 최댓값 = 가장 큰 내접원 중심. 길쭉한 꼬리(옆면)를
        무시하고 둥근 윗부분 중심을 잡는다."""
        dist = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
        _, max_val, _, max_loc = cv2.minMaxLoc(dist)
        if max_val <= 0:
            return centroid, None
        center = np.array([float(max_loc[0]), float(max_loc[1])], dtype=np.float32)
        return center, float(max_val)

    def _top_hole(self, frame_bgr, binary, centroid):
        """윗면 도넛 홀(어두운 중앙 구멍)의 중심을 검출. 실패 시 None.

        조명 강건성 설계:
          1) 탐색 범위를 **윗면(top face)** 으로 한정 — 내접원 디스크 안만 본다.
             옆면 몸통의 그림자가 '어두운 영역'으로 오검출되는 걸 원천 차단.
          2) 밝기 임계를 **Otsu**(윗면 픽셀 히스토그램의 골)로 자동 결정 — 절대
             밝기가 아니라 림(밝음)/홀(어두움) 의 상대 분포로 갈라 조명 변화에 적응.
             Otsu 가 비정상으로 높을 때만 dark_percentile 상한으로 가드.
          3) 후보 홀을 **원형도+중심성+면적** 으로 점수화, 볼트구멍 등 작은 잡음과
             그림자(비원형)를 배제하고 가장 그럴듯한 중앙 큰 홀만 채택.
          4) 중심은 minEnclosingCircle 가 아니라 **무게중심(moments)** 으로 — 외곽
             한두 픽셀 노이즈에 덜 흔들린다.
        """
        # ── 1) 윗면 영역 = 내접원 디스크 ───────────────────────
        dist = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
        _, insc_r, _, insc_loc = cv2.minMaxLoc(dist)
        if insc_r < 4:
            return None
        face_center = np.array([float(insc_loc[0]), float(insc_loc[1])], np.float32)
        face_r = max(3.0, insc_r * self.top_hole_face_ratio)
        face = np.zeros_like(binary)
        cv2.circle(face, (int(face_center[0]), int(face_center[1])),
                   int(face_r), 255, -1)
        face = cv2.bitwise_and(face, binary)
        face_area = float(np.count_nonzero(face))
        if face_area < 30:
            return None

        # ── 2) 윗면 픽셀의 밝기(HSV V) → Otsu 임계 (조명 자동 적응) ──
        v = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)[:, :, 2]
        vals = v[face > 0]
        if vals.size < 30:
            return None
        otsu_thr, _ = cv2.threshold(
            vals.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # Otsu 가 너무 높게 잡혀 윗면 대부분을 '어둡다'고 하면 가드.
        cap = float(np.percentile(vals, self.top_hole_dark_percentile))
        thr = min(float(otsu_thr), cap)

        dark = ((v <= thr) & (face > 0)).astype(np.uint8) * 255
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        cnts, _ = cv2.findContours(
            dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None

        # ── 3) 후보 홀 점수화: 원형도 + 중심성 + 면적 ──────────
        min_a = self.top_hole_min_area_frac * face_area
        max_a = self.top_hole_max_area_frac * face_area
        best = None
        for c in cnts:
            a = float(cv2.contourArea(c))
            if a < max(min_a, 30.0) or a > max_a:
                continue
            (cu, cv_), cr = cv2.minEnclosingCircle(c)
            if cr < 2:
                continue
            circularity = a / (math.pi * cr * cr)
            if circularity < self.top_hole_min_circularity:
                continue
            Mh = cv2.moments(c)
            if abs(Mh["m00"]) < 1e-6:
                continue
            hx = Mh["m10"] / Mh["m00"]
            hy = Mh["m01"] / Mh["m00"]
            dist_c = math.hypot(hx - face_center[0], hy - face_center[1])
            if dist_c > face_r:                 # 윗면 밖 중심은 제외
                continue
            # 핵심: pick 대상(컵 입구/center 구멍)은 rim 안에서 **가장 큰** 어두운
            # 영역이고 볼트 구멍은 작다. 따라서 면적을 지배적 가중치로 두고, 중심성은
            # 가장자리 그림자만 약하게 깎는 보조항으로 쓴다(원형도는 hard gate).
            #   score = area × (1 − k·(dist/face_r)²)   (k=top_hole_centrality_penalty)
            r = dist_c / face_r
            score = a * (1.0 - self.top_hole_centrality_penalty * r * r)
            if best is None or score > best[0]:
                best = (score, hx, hy, cr)
        if best is None:
            return None
        center = np.array([best[1], best[2]], dtype=np.float32)
        return center, float(best[3])

    def _hough_circle(self, frame_bgr, contour, centroid):
        """contour bbox ROI 안에서 HoughCircles 로 rim 원을 직접 검출.
        검출 실패 시 None (호출부가 내접원으로 폴백)."""
        x, y, w, h = cv2.boundingRect(contour)
        if min(w, h) < 4:
            return None
        pad = int(0.15 * max(w, h))
        H, W = frame_bgr.shape[:2]
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(W, x + w + pad), min(H, y + h + pad)
        roi = frame_bgr[y0:y1, x0:x1]
        if roi.size == 0:
            return None
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        half_short = max(2.0, min(w, h) / 2.0)
        min_r = max(1, int(self.hough_min_radius_ratio * half_short))
        max_r = max(min_r + 1, int(self.hough_max_radius_ratio * half_short))
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=self.hough_dp,
            minDist=half_short,
            param1=self.hough_param1, param2=self.hough_param2,
            minRadius=min_r, maxRadius=max_r)
        if circles is None:
            return None
        circles = np.asarray(circles, dtype=np.float32).reshape(-1, 3)
        # contour 안에 중심이 들어오는 원 중 가장 큰 것을 고른다.
        best = None
        for cu, cv_, cr in circles:
            gu, gv = float(cu) + x0, float(cv_) + y0
            if cv2.pointPolygonTest(contour, (gu, gv), False) < 0:
                continue
            if best is None or cr > best[2]:
                best = (gu, gv, float(cr))
        if best is None:
            return None
        return np.array([best[0], best[1]], dtype=np.float32), best[2]

    def _class_id_to_name(self, cls_id):
        if cls_id is None or cls_id < 0:
            return None
        names = getattr(self.model, "names", None)
        if names is None:
            return None
        if isinstance(names, dict):
            return names.get(cls_id)
        try:
            return names[cls_id]
        except (IndexError, KeyError, TypeError):
            return None

    def filter_target_detections(self, detections):
        if not self.target_class_name:
            return detections
        return [d for d in detections if d.get("cls_name") == self.target_class_name]

    # ── 색 분류 ───────────────────────────────────────────────
    def detect_color(self, frame_bgr, det):
        if self.cup_color:
            return self.cup_color
        mask = det["mask"] > 0
        if not np.any(mask):
            return "unknown"
        mean_bgr = frame_bgr[mask].mean(axis=0)
        return classify_color_bgr(mean_bgr)

    # ── 좌표 변환 (camera optical → base_link) ────────────────
    def cam_to_base(self, p_cam):
        """p_cam (camera optical frame, m) → base_link (m). 실패 시 None."""
        try:
            T_base_ee = get_ee_matrix(self.robot)
        except Exception as e:
            self.get_logger().warn(f"get_ee_matrix 실패(FK 불가): {e}")
            return None
        T_base_cam = T_base_ee @ self.gripper2cam
        p_base = (T_base_cam @ np.append(np.asarray(p_cam, dtype=float), 1.0))[:3]
        p_base = p_base - self.base_offset
        return p_base

    # ── Main callback ─────────────────────────────────────────
    def image_callback(self, msg: Image):
        try:
            frame_bgr = imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"image conversion failed: {e}")
            return

        h, w = frame_bgr.shape[:2]
        debug = frame_bgr.copy()
        start = time.time()

        try:
            with torch.inference_mode():
                results = self.model.predict(
                    source=frame_bgr,
                    imgsz=self.imgsz,
                    conf=self.conf,
                    iou=self.iou,
                    device=self.device,
                    half=self.half,
                    verbose=False,
                    retina_masks=True,
                )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        detections = self.extract_detections(results[0], frame_bgr, h, w)
        targets = self.filter_target_detections(detections)

        cups = []  # [{"xy_base":(x,y), "z_base":z, "color":str, "center":(u,v)}]
        for det in targets:
            u, v = det["center"]
            z = self.get_depth_at_pixel(u, v, window=7)
            if z is None:
                continue
            p_cam = self.deproject_pixel_to_3d(u, v, z)
            if p_cam is None:
                continue
            p_base = self.cam_to_base(p_cam)
            if p_base is None:
                continue
            color = self.detect_color(frame_bgr, det)
            cups.append({
                "xy_base": (float(p_base[0]), float(p_base[1])),
                "z_base": float(p_base[2]),
                "color": color,
                "center": (float(u), float(v)),
            })

        # 시간 평활/트래킹 (base_link 공간): per-frame 튐·outlier 제거.
        if self.enable_temporal_smoothing:
            published = self.tracker.update(cups, time.time())
        else:
            published = cups
        self.publish_boxes(published)

        # ── debug 시각화 ──
        for det in targets:
            cv2.drawContours(debug, [det["contour"]], -1, (0, 200, 255), 1)
            # 검출된 원(내접원/hough) — 초록 테두리
            if det.get("pick_radius"):
                c = det["center"]
                cv2.circle(debug, (int(c[0]), int(c[1])),
                           int(det["pick_radius"]), (0, 255, 0), 2)
            # 기존 무게중심(회색) vs 최종 pick point(빨강) 비교
            ctr = det.get("centroid")
            if ctr is not None:
                cv2.circle(debug, (int(ctr[0]), int(ctr[1])), 3, (160, 160, 160), -1)
            c = det["center"]
            cv2.circle(debug, (int(c[0]), int(c[1])), 4, (0, 0, 255), -1)
        cv2.putText(
            debug,
            f"upright cups={len(targets)} published={len(published)} "
            f"pick={self.pick_point_method} smooth={self.enable_temporal_smoothing}",
            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.publish_debug(debug, msg.header)

        elapsed = (time.time() - start) * 1000.0
        self.get_logger().info(
            f"upright cups={len(targets)} base={len(cups)} published={len(published)} "
            f"time={elapsed:.1f} ms")

    # ── Publish ───────────────────────────────────────────────
    def publish_boxes(self, cups):
        """모든 컵을 base_link MarkerArray 로 발행 (fake_hand_eye 형식)."""
        markers = MarkerArray()

        # 스냅샷 초기화: 구독자(_boxes dict)가 이전 프레임 잔재를 안 들고 있게.
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.ns = "box_top"
        markers.markers.append(clear)

        now = self.get_clock().now().to_msg()
        for i, cup in enumerate(cups):
            x, y = cup["xy_base"]
            z = cup["z_base"]
            color = cup["color"]
            mid = int(cup.get("id", i))   # 트래커 안정 id(있으면) 사용

            top = Marker()
            top.header.frame_id = self.base_frame
            top.header.stamp = now
            top.ns = "box_top"
            top.id = mid
            top.action = Marker.ADD
            top.type = Marker.SPHERE
            top.pose.position.x = x
            top.pose.position.y = y
            top.pose.position.z = z
            top.pose.orientation.w = 1.0
            markers.markers.append(top)

            label = Marker()
            label.header.frame_id = self.base_frame
            label.header.stamp = now
            label.ns = "box_labels"
            label.id = mid
            label.action = Marker.ADD
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z
            label.pose.orientation.w = 1.0
            label.text = f"#{mid}_c={color}_upright-cup"
            markers.markers.append(label)

        self.boxes_pub.publish(markers)

    def publish_debug(self, image_bgr, header):
        try:
            out = cv2_to_imgmsg(image_bgr, encoding="bgr8")
            out.header = header
            self.debug_pub.publish(out)
        except Exception as e:
            self.get_logger().warn(f"debug image publish failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UprightCupPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
