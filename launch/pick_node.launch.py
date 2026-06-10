from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── 토픽 ──────────────────────────────
        DeclareLaunchArgument(
            "move_result_topic", default_value="/move_result",
            description="plan_executor coarse move 완료 신호(+slot, +타깃 x,y). "
                        "std_msgs/String JSON.",
        ),
        DeclareLaunchArgument(
            "hand_eye_boxes_topic", default_value="/hand_eye/boxes",
            description="hand-eye 비전(또는 fake_hand_eye)이 내는 컵 마커. "
                        "visualization_msgs/MarkerArray, base_link frame.",
        ),
        DeclareLaunchArgument(
            "action_result_topic", default_value="/action_result",
            description="pyramid 완료 결과를 GSP/fake_hand_eye 로. std_msgs/String JSON.",
        ),
        # ── pyramid API ───────────────────────
        DeclareLaunchArgument(
            "api_base", default_value="https://yarr-api-31.simplyimg.com",
        ),
        DeclareLaunchArgument(
            "api_path", default_value="/api/robot/skill/pyramid",
        ),
        DeclareLaunchArgument(
            "api_timeout_sec", default_value="180.0",
            description="pyramid 스킬은 실팔(move+pick+place)을 끝까지 돌린 뒤 응답. "
                        "스킬 수행시간보다 길어야 함(짧으면 409).",
        ),
        # ── 컵 선택 ───────────────────────────
        DeclareLaunchArgument(
            "box_wait_sec", default_value="1.5",
            description="/hand_eye/boxes 마커 수집 대기(s). publish 주기 이상으로.",
        ),
        DeclareLaunchArgument("box_top_ns", default_value="box_top"),
        DeclareLaunchArgument("box_labels_ns", default_value="box_labels"),
        DeclareLaunchArgument(
            "filter_by_color", default_value="true",
            description="true 면 move_result.color 와 box_labels 색이 맞는 컵만 후보로.",
        ),
        # ── 트리거 보조 게이트 ────────────────
        DeclareLaunchArgument(
            "require_result_success", default_value="false",
            description="true 면 move_result.result 가 success_result_values 중 하나여야 동작. "
                        "1차 게이트(유효 slot)는 항상 적용됨.",
        ),
        DeclareLaunchArgument(
            "success_result_values", default_value="success,ok,200,true,done",
        ),
        DeclareLaunchArgument(
            "trigger_actions", default_value="",
            description="콤마구분. 비우면 모든 action 허용. 예: 'pyramid'.",
        ),

        Node(
            package="hand_pick",
            executable="pick_node",
            name="pick_node",
            output="screen",
            parameters=[{
                "move_result_topic": LaunchConfiguration("move_result_topic"),
                "hand_eye_boxes_topic": LaunchConfiguration("hand_eye_boxes_topic"),
                "action_result_topic": LaunchConfiguration("action_result_topic"),
                "api_base": LaunchConfiguration("api_base"),
                "api_path": LaunchConfiguration("api_path"),
                "api_timeout_sec": LaunchConfiguration("api_timeout_sec"),
                "box_wait_sec": LaunchConfiguration("box_wait_sec"),
                "box_top_ns": LaunchConfiguration("box_top_ns"),
                "box_labels_ns": LaunchConfiguration("box_labels_ns"),
                "filter_by_color": LaunchConfiguration("filter_by_color"),
                "require_result_success": LaunchConfiguration(
                    "require_result_success"),
                "success_result_values": LaunchConfiguration(
                    "success_result_values"),
                "trigger_actions": LaunchConfiguration("trigger_actions"),
            }],
        ),
    ])
