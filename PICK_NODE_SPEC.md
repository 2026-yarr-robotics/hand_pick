# pick_node 명세 (PICK_NODE_SPEC)

> hand-eye 컵 좌표로 pyramid API를 호출하는 ROS2 노드.
> 이 문서는 살아있는 명세다 — 인터페이스가 확정되면 계속 보완한다.

최종 수정: 2026-06-05 (cup-stack-integration **v1.1** 계약에 맞춰 개정)

> **v1.1 개정 핵심**: 상류 통합 repo
> (github.com/2026-yarr-robotics/cup-stack-integration, tag v1.1)를 확인하여
> 인터페이스를 추정→확정으로 교체했다. 바뀐 점:
> 1. 결과 토픽 `/pick_action_result` → **`/action_result`** (GSP·fake_hand_eye 가 구독).
>    페이로드 스키마도 GSP 계약에 맞춤 (아래 §3.2).
> 2. pick 입력 `/upright_cup/grasp_pose`(camera frame PoseStamped) →
>    **`/hand_eye/boxes`**(visualization_msgs/MarkerArray, **base_link frame**).
>    카메라→base 변환은 **hand-eye 비전 노드 책임**으로 이동(아래 §5·§6).
> 3. pick_node 는 변환을 안 하고, base_link 마커 중 **EE 최근접 컵**을 고른다.

---

## 1. 목적 / 배경

외란에 강건한 speed-stack 파이프라인 (통합 repo v1.1):

```
fake_aggregator ─/cups_on_table,/stack─┐
fake_digital_twin ─/digital_twin/boxes,/stack_track_ids─┐
                                        ▼
goal_state_publisher(GSP) ─/llm_input→ llm_node ─/llm_output→ plan_executor
                                                                  │
                          coarse move: POST /api/robot/move {x,y,z}│
                                                                  ▼
                                          /move_result (slot 포함)
                                                                  ▼
                                              ┌──────── pick_node ────────┐
                       /hand_eye/boxes ──────▶│ EE 최근접 컵 (x,y) 선택     │
                       (fake_hand_eye /        │ POST /skill/pyramid {x,y,slot}
                        실로봇 hand-eye 비전)   └────────────┬───────────────┘
                                                            ▼
                                              /action_result → GSP, fake_hand_eye
```

**2-stage pick (v1.1)**: 예전엔 plan_executor 가 exo-view 좌표로 직접
`skill/pyramid` 를 호출했는데 exo 좌표가 부정확했다. v1.1 은 coarse→fine 으로 분리:
- **plan_executor**: color→exo-view 컵 XY 해소 후 `POST /api/robot/move {x,y,z}` 로
  팔을 컵 위로 **대강** 이동(z=고정 접근 높이). skill 서버는 건드리지 않는다.
- **pick_node**(우리): move 성공(/move_result) 후 hand-eye 기반 정밀 컵 (x,y) 로
  `POST /api/robot/skill/pyramid` 를 **직접** 호출하고, 완료 신호 `/action_result` 도
  우리가 낸다(컵이 실제로 놓이는 시점은 우리 pyramid 호출이므로).

---

## 2. 파이프라인 타이밍

```
1. plan_executor 가 /api/robot/move 로 팔을 타깃 컵 위로 coarse 이동.
2. move 성공 → /move_result 발행 (성공이면 slot 포함, 실패면 slot 없음).
3. /move_result 에 유효 API slot 존재 == coarse 이동 완료 신호.
4. pick_node 가 /hand_eye/boxes(base_link)에서 EE 최근접 컵 (x,y) 선택.
5. POST /api/robot/skill/pyramid {x,y,slot}  ← 서버가 move→pick→place 수행.
   (서버는 cup release/place 시점에 200 반환, 최종 lift 는 계속될 수 있음.)
6. HTTP 200 & success → /action_result 발행 → GSP 가 in-flight LLM 트리거+플랜 진행,
   fake_hand_eye 는 disturbance 동기화.
```

> **busy 게이트(상류 사정)**: plan_executor 는 *다음* move 전에 skill_api_node
> status `busy=false` 를 기다린다. pick_node 의 pyramid 호출 자체는 이 게이트와
> 무관(서버가 첫 호출 시 lazy 기동). pick_node 는 신경 안 써도 됨.

---

## 3. ROS2 인터페이스

### 3.1 구독 (입력)

| 토픽 (기본값) | 타입 | 의미 |
|---|---|---|
| `/move_result` | `std_msgs/String` (JSON) | **트리거 + slot.** plan_executor coarse move 결과. |
| `/hand_eye/boxes` | `visualization_msgs/MarkerArray` | hand-eye 컵 후보. **base_link frame.** `box_top` ns=좌표, `box_labels` ns=색 텍스트. 실로봇=hand-eye 비전 노드, sim=fake_hand_eye_node. |

`/move_result` JSON (plan_executor `_publish_move_result` 기준):
```json
// 성공
{ "step": 1, "action": "pyramid", "color": "blue", "result": "success", "slot": "1l" }
// 실패 (slot 없음 → pick_node 가 무시)
{ "step": 1, "action": "pyramid", "color": "blue", "result": "fail", "failure_reason": "..." }
```
- **1차 트리거 게이트 = 유효한 `slot` 존재** (∈ `1l,1m,1r,2l,2r,3m`).
  실패 메시지엔 slot 이 없으므로 자동으로 걸러진다.
- 보조 게이트(파라미터 on/off): `action` 화이트리스트, `result` 성공값 확인.
- `slot` 은 **API slot**(`1l`). LLM canonical(`L1_left`)은 plan_executor 가 이미
  매핑해서 보냄.

`/hand_eye/boxes` 마커 (fake_hand_eye / digital_twin 과 동일 형태):
- `box_top` 마커: `id`=track_id, `pose.position.{x,y}` = 컵 base_link 좌표.
- `box_labels` 마커: `id`=track_id, `text` 예 `#5_slot=L2_right_c=blue_upright-cup`.
  pick_node 는 `c=<color>` 만 파싱(색 필터용). slot 라벨은 fake 전용이라 안 씀.

### 3.2 발행 (출력)

| 토픽 (기본값) | 타입 | 의미 |
|---|---|---|
| `/action_result` | `std_msgs/String` (JSON) | pyramid 완료/실패. **GSP** 와 **fake_hand_eye** 가 구독. |

`/action_result` JSON 페이로드:
```json
{
  "step": 1,
  "action": "pyramid",
  "color": "blue",
  "result": "success",
  "target_slot": "L1_left",
  "slot": "1l",
  "x": 0.2503,
  "y": -0.1998,
  "http_status": 200,
  "detail": "slot=1l pick=(...) place=(...)",
  "error": null
}
```
- **GSP 계약상 필수 키** (없으면 in-flight LLM 루프가 멈춤):
  - `result`: `"success"` | `"fail"` — GSP `_on_action_result` 가 문자열로 비교.
  - `action`: `"pyramid"` — GSP 가 world-reflection 게이팅에 사용.
  - `step`: int — GSP 가 `remaining_steps[0].step` 와 맞춰 플랜을 진행.
  - `color`: str — held color 추적 + `action_result_reflected` 비교.
  - `target_slot`: **canonical**(`L1_left`) — GSP `action_result_reflected` 가
    stack(canonical 키)과 대조. pick_node 가 API slot→canonical 역매핑해서 넣는다.
    fake_hand_eye 도 이 값으로 disturbance trigger 판정.
- 실패도 `result:"fail"` 로 발행 (`error` 에 사유: `select_failed`, `HTTP 5xx`,
  `success=false`, 네트워크 예외 문자열, `exception: ...`).

> **주의**: `success` 불리언이 아니라 `result` 문자열이 계약 키다. (v1.0 의
> `/pick_action_result {success:true}` 는 GSP 가 못 알아들어 루프가 정지했었음.)

---

## 4. pyramid API 계약

`POST {api_base}{api_path}` (기본 `https://yarr-api-31.simplyimg.com/api/robot/skill/pyramid`)

요청 body (`PyramidSkillRequest`):
| 필드 | 타입 | 설명 |
|---|---|---|
| `x` | number | pick 컵 X (base_link, m) |
| `y` | number | pick 컵 Y (base_link, m) |
| `slot` | enum | `1l/1m/1r`(bottom), `2l/2r`(mid), `3m`(top) — **API slot** |

> center · degree(yaw) · pick_z 는 서버가 `/api/robot/config/pyramid` 저장값을
> 자동 주입하므로 본문에 안 넣는다. 서버는 제공된 x,y 를 그대로 pick 타깃으로 신뢰하고,
> slot 키를 내부 pyramid geometry 로 place pose 변환. cup release/place 시 200 반환.

통합 repo README 의 측정 좌표 기준 기대 body (참고):
```json
{"x":0.250,"y":-0.20,"slot":"1l"} {"x":0.250,"y":0.00,"slot":"1m"} {"x":0.250,"y":0.20,"slot":"1r"}
{"x":0.350,"y":-0.20,"slot":"2l"} {"x":0.350,"y":0.00,"slot":"2r"} {"x":0.350,"y":0.20,"slot":"3m"}
```

응답 200 (`PyramidSkillResponse`):
```json
{ "success": true, "skill": "pyramid", "detail": "slot=1l pick=(...) place=(...)" }
```

---

## 5. 컵 선택 (hand-eye, base_link)

pick_node 는 **좌표 변환을 하지 않는다** — `/hand_eye/boxes` 마커가 이미 base_link 다.
대신 **현재 EE(그리퍼) 위치에 가장 가까운 컵**을 고른다:

```
ee_xy   = get_ee_matrix(robot)[:2, 3]      # MoveItPy 실시간 FK (link_6 global transform)
cands   = box_top 마커들의 (x,y)            # color 필터(move_result.color) 적용 가능
pick    = argmin_cup ‖cup_xy - ee_xy‖       # EE 최근접 컵
x, y    = pick                              # 그대로 pyramid body 로
```

근거: plan_executor 가 coarse move 로 EE 를 타깃 컵 위에 올려놨고, exo 오차(0.02m)가
컵 간격(0.10m)의 절반보다 작아 각 perturbed pose 가 자기 true 컵에 최근접이다
(fake_digital_twin 주석과 동일 전제). 오프라인 수치검증 통과(6컵 모두 정상 선택).

- `get_ee_matrix` 는 `stand_fallen_cup.py` 의 검증된 FK 재사용 (이번엔 변환이 아니라
  **선택 기준점**으로만 사용). 상수: `EE_LINK=link_6`, `GROUP_NAME=manipulator`.
- `filter_by_color`(기본 true): `move_result.color` 와 `box_labels` 색이 같은 컵만 후보.
  fake 는 전부 blue 라 색 필터만으로는 구분 안 됨 → EE 최근접이 실제 선택자.
- sim 모드면 FK/마커 우회, `sim_pick_x/y` 사용.

---

## 6. hand-eye 비전 노드 (`/hand_eye/boxes` 생산자 — 별도 작업)

v1.1 에서 카메라→base 변환은 **비전 노드 책임**이다 (통합 repo 의 fake_hand_eye 가
이미 base_link 마커를 내고, "실제 hand-eye 비전 노드가 fake 를 그대로 대체"하는 게 설계).
즉 비전 노드가:
1. hand-eye 이미지에서 upright 컵 윗면 중앙 픽셀 검출(YOLO-seg, class `upright-cup`),
2. depth deproject → camera optical frame 3D,
3. **카메라→base_link 변환**(`stand_fallen_cup.py` 의 `T_base_cam = T_base_ee @ T_ee_cam`,
   `T_gripper2camera.npy`, MoveItPy FK) 적용,
4. `/hand_eye/boxes`(MarkerArray, base_link, `box_top`+`box_labels`)로 발행.

> **구현 완료(2026-06-05)**: `speed_stack_yolo_seg/upright_cup_pose_node.py` 가 위
> 1~4 를 모두 수행한다 — MoveItPy FK(`get_ee_matrix(link_6)`) + `T_gripper2camera.npy`
> (mm→m) 로 카메라→base 변환 후, 검출된 모든 upright 컵을 base_link `/hand_eye/boxes`
> MarkerArray(`box_top`=좌표, `box_labels`=`#i_c=<color>_upright-cup`)로 발행.
> 매 프레임 `DELETEALL` 먼저 보내 스냅샷을 갱신한다. 색은 mask 평균색 HSV 분류(또는
> `cup_color` 파라미터로 고정). 즉 fake_hand_eye_node 의 실(real) 대체물이며 pick_node
> 는 fake/real 구분 없이 동일 동작. (실카메라 없어 좌표 정밀도 end-to-end 검증은 보류 —
> 코드/변환식 레벨 검증만 완료.)

> 비전 노드 없이 pick_node 만 테스트: sim 은 `fake_hand_eye_node.py`(통합 repo)로
> `/hand_eye/boxes` 를 흘리거나, `sim:=true` + `sim_pick_x/y` 로 대체.

---

## 7. 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `move_result_topic` | `/move_result` | 트리거+slot 입력 토픽 |
| `hand_eye_boxes_topic` | `/hand_eye/boxes` | base_link 컵 MarkerArray 입력 |
| `action_result_topic` | `/action_result` | 결과 출력 토픽 (GSP/fake_hand_eye) |
| `api_base` | `https://yarr-api-31.simplyimg.com` | API 호스트 |
| `api_path` | `/api/robot/skill/pyramid` | API 경로 |
| `api_timeout_sec` | `10.0` | HTTP 타임아웃(s) |
| `box_wait_sec` | `1.5` | `/hand_eye/boxes` 수집 대기(s, publish 주기 이상) |
| `box_top_ns` / `box_labels_ns` | `box_top` / `box_labels` | 마커 namespace |
| `filter_by_color` | `true` | `move_result.color` 로 후보 컵 필터 |
| `require_result_success` | `false` | true 면 `result` 성공값까지 확인 |
| `success_result_values` | `success,ok,200,true,done` | result 성공값 후보(콤마구분, 소문자) |
| `trigger_actions` | `""` | action 화이트리스트(콤마구분). 비우면 전체 허용 |
| `sim` | `false` | 컵 선택·MoveItPy 우회 |
| `sim_pick_x` / `sim_pick_y` | `0.40` / `0.10` | sim pick 좌표(base_link) |

---

## 8. 빌드 / 실행

### 빌드
```bash
yolo_build --packages-select pick_node
```
`.bashrc` 의 `yolo_build` 가 colcon 빌드 후 entry-point shebang 을 `.venv` python 으로
교정한다 (entry_points 배열에 pick_node 줄 이미 등록됨).

### 실행 (실로봇)
```bash
# Term1: MoveIt bringup (FK 용)
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m0609 host:=192.168.1.100
# Term2: realsense (hand-eye 카메라)
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true align_depth.enable:=true ...
# Term3: hand-eye 비전 노드 (별도 작업; /hand_eye/boxes base_link 발행)
# Term4: pick_node
ros2 launch pick_node pick_node.launch.py
```

### 실행 (sim — 비전/로봇 없이 API만)
```bash
ros2 launch pick_node pick_node.launch.py sim:=true sim_pick_x:=0.40 sim_pick_y:=0.10
```

### 실행 (통합 sim — fake_hand_eye + 실로봇 FK)
fake_hand_eye 가 6컵을 base_link 로 흘리므로 EE 최근접 선택 검증엔 FK(실로봇/MoveItPy)가
필요하다. fake 노드들은 통합 repo `cup_stack_agent/start.sh` 로 띄운다(pick_node 는 미포함).

---

## 9. 검증 (code-level — API 미연결 상태)

1. **sim 스키마**: `sim:=true api_base:=http://127.0.0.1:9` 기동 후
   ```bash
   ros2 topic pub --once /move_result std_msgs/String \
     '{data: "{\"step\":2,\"action\":\"pyramid\",\"color\":\"blue\",\"result\":\"success\",\"slot\":\"2r\"}"}'
   ros2 topic echo /action_result
   ```
   → `/action_result` 에 `{result, action:"pyramid", color, target_slot:"L2_right",
   slot:"2r", x, y, ...}` 발행 확인. **(통과 2026-06-05)** dead API 라 result:"fail".
2. **선택 로직(오프라인)**: fake MEASURED_CUPS 6개 + perturbed EE → EE 최근접이
   해당 컵을 고르는지 수치검증. **(6/6 통과)**
3. **slot 역매핑**: `1l→L1_left … 3m→L3_top`. **(통과)**
4. **실로봇/실API (미수행)**: bringup + 비전 + `/move_result` 흐름에서 base (x,y) 가
   실측 컵과 ±2cm, HTTP 200, `result:"success"`, GSP in-flight 루프 진행 확인 필요.

---

## 10. 알려진 한계 / TODO

- [x] hand-eye 비전 노드를 v1.1 계약(base_link `/hand_eye/boxes` MarkerArray)으로 구현(§6).
      `upright_cup_pose_node.py` 가 카메라→base 변환 + base MarkerArray 발행하도록 재작성(2026-06-05).
- [ ] 실로봇/실API end-to-end (현재 API 미연결 — code-level 검증만 완료).
- [ ] color 구분 pick: 비전 노드가 mask 평균색 HSV 분류로 `box_labels` 에 `c=<color>` 발행.
      조명/재질 따라 오분류 가능 → 실환경에서 임계값 튜닝 필요(또는 `cup_color` 고정).
- [ ] EE 최근접 선택은 coarse move 정확도에 의존(exo 오차 < 컵간격/2 가정). 컵이 촘촘하면 재검토.
- [ ] select 실패 / API 실패 시 재시도 정책 (현재 1회 시도 후 실패 발행).
- [ ] 동시 다발 `/move_result` 는 busy 중 드롭. 큐잉 필요하면 도입.
- [ ] 통합 후 push: github.com/2026-yarr-robotics/cup-stack-integration 에 합류 예정.
