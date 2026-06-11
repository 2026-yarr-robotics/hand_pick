from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("hand_pick")
    default_weights = os.path.join(pkg_share, "weights", "best.pt")

    return LaunchDescription([
        DeclareLaunchArgument(
            "weights_path",
            default_value=default_weights,
            description="YOLO-seg 가중치(.pt). repo 미포함 — share/weights/best.pt 에 "
                        "두거나 이 인자로 경로 지정.",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/camera/color/image_raw",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info",
        ),
        DeclareLaunchArgument(
            "boxes_topic",
            default_value="/hand_eye/boxes",
            description="base_link MarkerArray (box_top + box_labels). "
                        "pick_node 가 구독. fake_hand_eye 대체.",
        ),
        DeclareLaunchArgument(
            "debug_image_topic", default_value="/upright_cup/debug_image"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("iou", default_value="0.45"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("half", default_value="false"),
        DeclareLaunchArgument(
            "target_class_name",
            default_value="upright-cup",
            description="pick 대상 YOLO 클래스 이름. 이 클래스 mask 만 사용.",
        ),
        DeclareLaunchArgument("min_mask_area", default_value="300.0"),
        DeclareLaunchArgument(
            "dedup_min_dist_px", default_value="25.0",
            description="pick point 가 이 거리(px) 안인 같은 클래스 중복 검출은 "
                        "conf 높은 것만 남김. 0 이하면 비활성.",
        ),
        # ── 시간 평활/트래킹 (base_link) ──
        DeclareLaunchArgument(
            "enable_temporal_smoothing", default_value="true",
            description="프레임 간 컵 추적 후 EMA 평활 + outlier 제거. per-frame 튐 억제.",
        ),
        DeclareLaunchArgument("track_match_dist", default_value="0.08"),
        DeclareLaunchArgument("smoothing_alpha", default_value="0.4"),
        DeclareLaunchArgument("track_outlier_dist", default_value="0.04"),
        DeclareLaunchArgument("track_reacquire_frames", default_value="4"),
        DeclareLaunchArgument("track_timeout_sec", default_value="0.5"),
        DeclareLaunchArgument("track_min_hits", default_value="2"),
        # ── pick point: mask 에서 "원(rim)" 중심 산출 방식 ──
        DeclareLaunchArgument(
            "pick_point_method", default_value="inscribed",
            description="top_hole(윗면 도넛 홀 중심) | inscribed(내접원, 기본) | "
                        "hough(원 직접검출) | centroid(기존 무게중심). 컵 입구/관통홀 "
                        "정밀 pick 은 top_hole, 일반 강건 기본은 inscribed.",
        ),
        DeclareLaunchArgument("hough_dp", default_value="1.2"),
        DeclareLaunchArgument("hough_param1", default_value="100.0"),
        DeclareLaunchArgument("hough_param2", default_value="25.0"),
        DeclareLaunchArgument("hough_min_radius_ratio", default_value="0.25"),
        DeclareLaunchArgument("hough_max_radius_ratio", default_value="0.75"),
        DeclareLaunchArgument("top_hole_face_ratio", default_value="2.5"),
        DeclareLaunchArgument("top_hole_min_circularity", default_value="0.45"),
        DeclareLaunchArgument("top_hole_dark_percentile", default_value="35.0"),
        DeclareLaunchArgument("top_hole_min_area_frac", default_value="0.01"),
        DeclareLaunchArgument("top_hole_max_area_frac", default_value="0.7"),
        DeclareLaunchArgument("top_hole_centrality_penalty", default_value="0.4"),
        # ── camera → base_link 변환 ──
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "calib_file",
            default_value="",
            description="hand-eye 캘리브 npy. 비우면 hand_pick share 의 "
                        "config/T_gripper2camera.npy 사용.",
        ),
        DeclareLaunchArgument(
            "calib_scale_mm_to_m", default_value="true",
            description="캘리브 translation 이 mm 면 true (÷1000).",
        ),
        DeclareLaunchArgument("base_offset_x", default_value="0.0"),
        DeclareLaunchArgument("base_offset_y", default_value="0.0"),
        DeclareLaunchArgument(
            "base_offset_z", default_value="0.080",
            description="hand-eye 잔여 z 오차 보정(m). stand_fallen_cup 과 동일.",
        ),
        DeclareLaunchArgument(
            "cup_color", default_value="",
            description="비우면 mask 평균색 자동 분류. 값 지정 시 전 컵 고정 색.",
        ),

        Node(
            package="hand_pick",
            executable="upright_cup_pose_node",
            name="upright_cup_pose_node",
            output="screen",
            parameters=[{
                "weights_path": LaunchConfiguration("weights_path"),
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),

                "boxes_topic": LaunchConfiguration("boxes_topic"),
                "debug_image_topic": LaunchConfiguration("debug_image_topic"),

                "imgsz": LaunchConfiguration("imgsz"),
                "conf": LaunchConfiguration("conf"),
                "iou": LaunchConfiguration("iou"),
                "device": LaunchConfiguration("device"),
                "half": LaunchConfiguration("half"),

                "target_class_name": LaunchConfiguration("target_class_name"),
                "min_mask_area": LaunchConfiguration("min_mask_area"),
                "dedup_min_dist_px": LaunchConfiguration("dedup_min_dist_px"),
                "enable_temporal_smoothing": LaunchConfiguration("enable_temporal_smoothing"),
                "track_match_dist": LaunchConfiguration("track_match_dist"),
                "smoothing_alpha": LaunchConfiguration("smoothing_alpha"),
                "track_outlier_dist": LaunchConfiguration("track_outlier_dist"),
                "track_reacquire_frames": LaunchConfiguration("track_reacquire_frames"),
                "track_timeout_sec": LaunchConfiguration("track_timeout_sec"),
                "track_min_hits": LaunchConfiguration("track_min_hits"),

                "pick_point_method": LaunchConfiguration("pick_point_method"),
                "hough_dp": LaunchConfiguration("hough_dp"),
                "hough_param1": LaunchConfiguration("hough_param1"),
                "hough_param2": LaunchConfiguration("hough_param2"),
                "hough_min_radius_ratio": LaunchConfiguration("hough_min_radius_ratio"),
                "hough_max_radius_ratio": LaunchConfiguration("hough_max_radius_ratio"),
                "top_hole_face_ratio": LaunchConfiguration("top_hole_face_ratio"),
                "top_hole_min_circularity": LaunchConfiguration("top_hole_min_circularity"),
                "top_hole_dark_percentile": LaunchConfiguration("top_hole_dark_percentile"),
                "top_hole_min_area_frac": LaunchConfiguration("top_hole_min_area_frac"),
                "top_hole_max_area_frac": LaunchConfiguration("top_hole_max_area_frac"),
                "top_hole_centrality_penalty": LaunchConfiguration("top_hole_centrality_penalty"),

                "base_frame": LaunchConfiguration("base_frame"),
                "calib_file": LaunchConfiguration("calib_file"),
                "calib_scale_mm_to_m": LaunchConfiguration("calib_scale_mm_to_m"),
                "base_offset_x": LaunchConfiguration("base_offset_x"),
                "base_offset_y": LaunchConfiguration("base_offset_y"),
                "base_offset_z": LaunchConfiguration("base_offset_z"),
                "cup_color": LaunchConfiguration("cup_color"),
            }],
        ),
    ])
