# hand_pick

Hand-eye view 기반 정밀 pick 모듈 (ROS 2, `ament_python`).

고정 카메라(exo-view)는 pick 좌표가 부정확해서, **hand-eye 카메라**로 컵을 검출하고
그 좌표로 정밀 pick 을 수행한다. 두 노드로 구성된다.

```
카메라 ─▶ upright_cup_pose_node ─▶ /hand_eye/boxes (base_link MarkerArray)
                                          │
                       /move_result ─▶ pick_node ─▶ POST /api/robot/skill/pyramid
                                          │                    │
                                          └──────▶ /action_result ◀──────┘
```

## 노드

### `upright_cup_pose_node`
hand-eye 카메라로 똑바로 선(upright) 컵을 YOLO-seg 검출하고, **카메라 광학좌표 →
base_link 변환**까지 수행해 모든 컵을 `/hand_eye/boxes` 로 발행한다.
fake_hand_eye_node 의 실(real) 대체물.

- 좌표 변환: `T_base_cam = get_ee_matrix(robot) @ T_gripper2camera` (MoveItPy FK + hand-eye calib)
- 발행: `/hand_eye/boxes` (`visualization_msgs/MarkerArray`, base_link frame)
  - `ns=box_top`  : 컵 좌표 마커
  - `ns=box_labels` : `#i_c=<color>_upright-cup` 텍스트(색 라벨)
- 디버그: `/upright_cup/debug_image`

### `pick_node`
`/move_result`(plan_executor 의 coarse move 완료 + slot + 타깃 x,y) 를 트리거로,
`/hand_eye/boxes` 에서 **move 타깃에 가장 가까운 컵**의 (x,y) 를 골라
`POST /api/robot/skill/pyramid` 를 호출하고, 결과를 `/action_result` 로 발행한다.

- 입력 트리거: `/move_result` (`std_msgs/String` JSON) — 유효 slot = coarse 완료
- pick 입력: `/hand_eye/boxes` (base_link MarkerArray)
- 출력: `/action_result` (`std_msgs/String` JSON) — `result`, `action`, `step`, `color`, `target_slot`(canonical)

## 빌드

```bash
cd ~/ros2_ws/src && git clone https://github.com/2026-yarr-robotics/hand_pick.git
cd ~/ros2_ws && colcon build --packages-select hand_pick
source install/setup.bash
```

## 실행

```bash
# 비전 노드 (실카메라 + MoveItPy FK 필요)
ros2 launch hand_pick upright_cup_pose.launch.py weights_path:=/path/to/best.pt

# pick 노드
ros2 launch hand_pick pick_node.launch.py
```

## 전제 / 외부 의존성

- **MoveItPy(`moveit_py`)**: `upright_cup_pose_node` 의 link_6 FK 에 필요.
  `dsr_bringup2_moveit.launch.py` 가 떠 있어야 한다.
- **YOLO 가중치**: 용량 때문에 repo 에 미포함. `weights_path:=` 로 지정하거나
  `share/hand_pick/weights/best.pt` 에 배치.
- **hand-eye 캘리브**: `config/T_gripper2camera.npy` 포함(mm 단위). 다른 값은
  `calib_file:=` 로 지정.
- Python: `ultralytics`, `torch`, `opencv-python`, `requests`, `numpy`.

자세한 토픽/파라미터 계약은 [`PICK_NODE_SPEC.md`](PICK_NODE_SPEC.md) 참고.
