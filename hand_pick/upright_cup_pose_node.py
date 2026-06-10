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
    def extract_detections(self, result, image_h, image_w):
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
            conf = float(confs[i]) if confs is not None and i < len(confs) else 1.0
            cls_id = int(clss[i]) if clss is not None and i < len(clss) else -1
            detections.append({
                "mask": binary,
                "contour": contour,
                "area": area,
                "center": np.array([cx, cy], dtype=np.float32),
                "conf": conf,
                "cls_id": cls_id,
                "cls_name": self._class_id_to_name(cls_id),
            })
        return detections

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

        detections = self.extract_detections(results[0], h, w)
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

        self.publish_boxes(cups)

        # ── debug 시각화 ──
        for det in targets:
            cv2.drawContours(debug, [det["contour"]], -1, (0, 200, 255), 2)
            c = det["center"]
            cv2.circle(debug, (int(c[0]), int(c[1])), 4, (0, 200, 255), -1)
        cv2.putText(
            debug, f"upright cups={len(targets)} published={len(cups)}",
            (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.publish_debug(debug, msg.header)

        elapsed = (time.time() - start) * 1000.0
        self.get_logger().info(
            f"upright cups={len(targets)} published(base)={len(cups)} "
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

            top = Marker()
            top.header.frame_id = self.base_frame
            top.header.stamp = now
            top.ns = "box_top"
            top.id = i
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
            label.id = i
            label.action = Marker.ADD
            label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z
            label.pose.orientation.w = 1.0
            label.text = f"#{i}_c={color}_upright-cup"
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
