#!/usr/bin/env python3
"""
pick_node.py

Hand-eye 기반 정밀 pick 노드 (cup-stack-integration v1.1 계약).

파이프라인 위치:
    plan_executor (coarse move: POST /api/robot/move) → /move_result(slot 포함)
      → [pick_node] → POST /api/robot/skill/pyramid (서버가 move→pick→place)
      → /action_result → goal_state_publisher(GSP) / fake_hand_eye

동작 요약:
  1. /move_result (std_msgs/String, JSON) 구독. 유효한 API slot 이 들어오면 = 앞단
     plan_executor 의 coarse 이동이 끝났다는 신호. 본문의 x,y 가 그 coarse move
     타깃(= EE 가 대강 올라간 위치)이라, 컵 선택 기준으로 쓴다.
       성공 예: {"step":1,"action":"pyramid","color":"blue","result":"success",
                 "slot":"1l","x":0.26,"y":-0.18}
       실패 예: {"step":1,"action":"pyramid","color":"blue","result":"fail",
                 "failure_reason":"..."}  (slot 없음 → 무시)
  2. /hand_eye/boxes (visualization_msgs/MarkerArray, **base_link frame**) 에서
     컵 후보를 읽어, move_result 의 (x,y) 에 가장 가까운 컵의 (x,y) 를 고른다.
     (좌표 변환은 hand-eye 비전 노드가 담당 — 마커는 이미 base_link 좌표. 실제 EE
      를 MoveItPy FK 로 읽지 않고 move_result 의 coarse 타깃을 기준으로 쓴다 —
      로봇이 그 타깃으로 이동했으므로 EE ≈ move 타깃.)
  3. POST {api_base}{api_path} body {x, y, slot}. pick_z·center·yaw 는 서버가
     /api/robot/config/pyramid 에서 자동 주입하므로 본문에 안 넣는다.
  4. HTTP 200 & success=true → /action_result (std_msgs/String, JSON) 발행.
     GSP 가 in-flight LLM 트리거 + 플랜 진행에 쓰므로 스키마가 중요하다:
       {step, action:"pyramid", color, result:"success"|"fail", target_slot(canonical),
        slot(api), x, y, detail, error}
     실패도 result:"fail" 로 발행한다.

전제:
  - 컵 선택 기준 좌표는 /move_result 의 x,y (MoveItPy/로봇 bringup 불필요).
  - /hand_eye/boxes 발행 주체: 실로봇은 hand-eye 비전 노드, sim 은 fake_hand_eye_node.
"""

import json
import time

import numpy as np
import requests

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


# ─────────────────────────────────────────────────────────
#  상수 (stand_fallen_cup.py 와 동일 환경 가정)
# ─────────────────────────────────────────────────────────
VALID_SLOTS = ("1l", "1m", "1r", "2l", "2r", "3m")

# API slot(1l) → LLM canonical slot(L1_left). /move_result 는 API slot 만 주는데,
# GSP 의 action_result_reflected() 는 /action_result 의 target_slot 을 canonical
# 형태로 기대(stack 키가 L1_left)하므로 역매핑해서 같이 실어 준다.
# plan_executor_node._LLM_TO_API_SLOT 의 역.
_API_TO_LLM_SLOT = {
    "1l": "L1_left", "1m": "L1_mid", "1r": "L1_right",
    "2l": "L2_left", "2r": "L2_right",
    "3m": "L3_top",
}

# fake_digital_twin / fake_hand_eye 의 box_labels 텍스트에서 색을 뽑기 위한 사전.
_KNOWN_COLORS = frozenset({
    "red", "orange", "yellow", "green", "blue", "purple",
    "white", "black", "gray", "unknown",
})


# ─────────────────────────────────────────────────────────
#  유틸
# ─────────────────────────────────────────────────────────
def parse_label_color(text):
    """box_labels 마커 텍스트에서 color 추출 (예: '#5_slot=L2_right_c=blue_upright-cup').
    못 찾으면 None."""
    if not text:
        return None
    for tok in str(text).replace("\n", "_").split("_"):
        t = tok.strip().lower()
        if t.startswith("c=") and t[2:] in _KNOWN_COLORS:
            return t[2:]
    for tok in str(text).replace("\n", "_").split("_"):
        t = tok.strip().lower()
        if t in _KNOWN_COLORS:
            return t
    return None


# ─────────────────────────────────────────────────────────
#  Node
# ─────────────────────────────────────────────────────────
class PickNode(Node):
    def __init__(self):
        super().__init__("pick_node")
        log = self.get_logger()

        # ── 파라미터 선언 ──────────────────────────
        # 토픽
        self.declare_parameter("move_result_topic", "/move_result")
        self.declare_parameter("hand_eye_boxes_topic", "/hand_eye/boxes")
        self.declare_parameter("action_result_topic", "/action_result")
        # API
        self.declare_parameter("api_base", "https://yarr-api-31.simplyimg.com")
        self.declare_parameter("api_path", "/api/robot/skill/pyramid")
        # NOTE: the post-place arm retreat to HOME is done server-side inside
        # /skill/pyramid_step (skill_api_node calls try_move_home after the
        # place, and only then returns), so the exo camera is clear of the arm
        # by the time we publish /action_result. pick_node does not move home.
        # The pyramid skill runs the real arm (move+pick+place) and the server
        # only responds when it finishes, so this must exceed the skill duration.
        # Too short → pick_node gives up while the skill keeps running, and the
        # next call hits HTTP 409 "a skill is already running".
        self.declare_parameter("api_timeout_sec", 180.0)
        # 컵 선택
        self.declare_parameter("box_wait_sec", 1.5)   # 마커 수집 대기(>= publish 주기)
        self.declare_parameter("box_top_ns", "box_top")
        self.declare_parameter("box_labels_ns", "box_labels")
        self.declare_parameter("filter_by_color", True)  # move_result.color 로 후보 필터
        # 트리거 게이트 (1차 = 유효 slot 존재. 아래는 보조 필터)
        self.declare_parameter("require_result_success", False)
        self.declare_parameter("success_result_values", "success,ok,200,true,done")
        self.declare_parameter("trigger_actions", "")  # 빈값=모든 action 허용

        self.move_result_topic = str(self.get_parameter("move_result_topic").value)
        self.hand_eye_boxes_topic = str(
            self.get_parameter("hand_eye_boxes_topic").value)
        self.action_result_topic = str(
            self.get_parameter("action_result_topic").value)
        self.api_base = str(self.get_parameter("api_base").value).rstrip("/")
        self.api_path = str(self.get_parameter("api_path").value)
        self.api_url = self.api_base + self.api_path
        self.api_timeout_sec = float(self.get_parameter("api_timeout_sec").value)
        self.box_wait_sec = float(self.get_parameter("box_wait_sec").value)
        self.box_top_ns = str(self.get_parameter("box_top_ns").value)
        self.box_labels_ns = str(self.get_parameter("box_labels_ns").value)
        self.filter_by_color = bool(self.get_parameter("filter_by_color").value)
        self.require_result_success = bool(
            self.get_parameter("require_result_success").value)
        self.success_values = [
            v.strip().lower()
            for v in str(self.get_parameter("success_result_values").value).split(",")
            if v.strip()
        ]
        self.trigger_actions = [
            v.strip().lower()
            for v in str(self.get_parameter("trigger_actions").value).split(",")
            if v.strip()
        ]

        # 시작 배너·대기/게이트 로그는 debug 로 — 대시보드 피드에는 pick 시퀀스
        # (시작~종료) 동안의 로그만 보이도록 한다 (rclpy 기본 레벨 INFO → debug 는
        # 로그 파일/피드에 안 찍힘).
        log.debug("=== pick_node 시작 ===")
        log.debug(f"  trigger : {self.move_result_topic} (slot 게이트, move 타깃 x,y)")
        log.debug(f"  pick src: {self.hand_eye_boxes_topic} (base_link MarkerArray)")
        log.debug(f"  result  : {self.action_result_topic}")
        log.debug(f"  API     : POST {self.api_url} (timeout {self.api_timeout_sec}s)")

        # ── HTTP 세션 ──────────────────────────────
        self.http = requests.Session()

        # ── 상태 ───────────────────────────────────
        # 최신 /hand_eye/boxes 파싱 결과: id → {"xy": np.array, "color": str|None}
        self._boxes = {}
        self._pending = None   # 처리 대기 중인 move_result dict
        self._busy = False     # pyramid 시퀀스 처리 중 재진입 방지

        # ── pub/sub ────────────────────────────────
        self.result_pub = self.create_publisher(
            String, self.action_result_topic, 10)
        self.create_subscription(
            String, self.move_result_topic, self._move_result_cb, 10)
        self.create_subscription(
            MarkerArray, self.hand_eye_boxes_topic, self._boxes_cb, 10)

    # ── 콜백 ─────────────────────────────────────
    def _boxes_cb(self, msg: MarkerArray):
        """최신 /hand_eye/boxes 스냅샷 갱신. box_top=좌표, box_labels=색."""
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                self._boxes.clear()
                continue
            if m.action == Marker.DELETE:
                self._boxes.pop(m.id, None)
                continue
            entry = self._boxes.setdefault(m.id, {"xy": None, "color": None})
            if m.ns == self.box_top_ns:
                entry["xy"] = np.array(
                    [m.pose.position.x, m.pose.position.y], dtype=float)
            elif m.ns == self.box_labels_ns:
                c = parse_label_color(m.text)
                if c is not None:
                    entry["color"] = c

    def _move_result_cb(self, msg: String):
        """/move_result 수신. 유효 slot 이면 pending 으로 등록 (가벼운 게이트만).
        실제 pick 시퀀스는 run() 루프가 처리 — 콜백 안에서 blocking/spin 금지."""
        log = self.get_logger()
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            log.warn(f"[move_result] JSON 파싱 실패, 무시: {msg.data!r}")
            return
        if not isinstance(data, dict):
            log.warn(f"[move_result] dict 아님, 무시: {data!r}")
            return

        slot = str(data.get("slot", "")).strip().lower()
        if slot not in VALID_SLOTS:
            # slot 없음/무효 → coarse 이동 완료 신호 아님(=fail 등). 조용히 무시.
            log.debug(f"[move_result] slot 없음/무효({slot!r}) → 무시")
            return

        # 보조 필터: action
        if self.trigger_actions:
            action = str(data.get("action", "")).strip().lower()
            if action not in self.trigger_actions:
                log.debug(
                    f"[move_result] action={action!r} 가 trigger_actions 에 없음 → 무시")
                return

        # 보조 필터: result 성공값
        if self.require_result_success:
            result = str(data.get("result", "")).strip().lower()
            if result not in self.success_values:
                log.debug(f"[move_result] result={result!r} 성공값 아님 → 무시")
                return

        if self._busy or self._pending is not None:
            log.warn(f"[move_result] 처리 중(busy={self._busy}) — slot={slot} 드롭")
            return

        # 트리거 수신 로그도 debug — 피드는 "=== pick 시퀀스 시작 ===" 부터 시작.
        log.debug(f"[move_result] slot={slot} 수신 → pick 시퀀스 예약")
        self._pending = {"slot": slot, "raw": data}

    # ── 컵 선택 (hand-eye 마커 → base_link x,y) ────
    def _select_pick_xy(self, color, ref_xy):
        """ref_xy(=plan_executor 의 coarse move 타깃 x,y) 에 가장 가까운 hand-eye
        컵의 (x,y) 반환. 실패 시 None.

        plan_executor 가 coarse move 로 EE 를 타깃 컵 위에 대강 올려놓았으므로, 그
        move 타깃에 가장 가까운 hand-eye(참값) 컵 = 잡을 컵. MoveItPy 로 실제 EE 를
        읽을 필요 없이 move_result 의 x,y 를 기준으로 쓴다. (각 perturbed pose 가
        자기 true 컵에 최근접이라는 fake_digital_twin 전제와 동일.)"""
        log = self.get_logger()
        self._boxes.clear()

        log.info(f"[select] hand-eye 마커 수집 (max {self.box_wait_sec}s)")
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < self.box_wait_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

        cands = [(i, b) for i, b in self._boxes.items() if b.get("xy") is not None]
        if not cands:
            log.error(
                f"[select] 컵 마커 없음 ({self.hand_eye_boxes_topic} 흐르는지 확인)")
            return None

        if self.filter_by_color and color:
            cl = str(color).strip().lower()
            colored = [(i, b) for i, b in cands if b.get("color") == cl]
            if colored:
                cands = colored
            else:
                log.warn(
                    f"[select] color={cl!r} 매칭 컵 없음 → 전체 {len(cands)}개 중 선택")

        cid, best = min(
            cands, key=lambda ib: float(np.linalg.norm(ib[1]["xy"] - ref_xy)))
        x, y = float(best["xy"][0]), float(best["xy"][1])
        log.info(
            f"[select] move target=({ref_xy[0]:.3f},{ref_xy[1]:.3f}) → "
            f"cup#{cid} pick=({x:.3f},{y:.3f}) (후보 {len(cands)}개)")
        return np.array([x, y])

    # ── pyramid API 호출 ─────────────────────────
    def _call_pyramid(self, x, y, slot):
        """POST /api/robot/skill/pyramid. 반환: (ok, http_status, resp_json, err)."""
        log = self.get_logger()
        body = {"x": float(x), "y": float(y), "slot": slot}
        log.info(f"[pyramid] POST {self.api_url} body={body}")
        try:
            resp = self.http.post(
                self.api_url, json=body, timeout=self.api_timeout_sec)
        except requests.RequestException as e:
            log.error(f"[pyramid] 요청 실패: {e}")
            return (False, None, None, str(e))

        status = resp.status_code
        try:
            rj = resp.json()
        except ValueError:
            rj = None

        if status != 200:
            log.error(f"[pyramid] HTTP {status}: {resp.text[:200]}")
            return (False, status, rj, f"HTTP {status}")

        success = bool(rj.get("success", False)) if isinstance(rj, dict) else False
        if not success:
            detail = rj.get("detail", "") if isinstance(rj, dict) else ""
            log.error(f"[pyramid] success=false detail={detail!r}")
            return (False, status, rj, detail or "success=false")

        log.info(f"[pyramid] 성공 HTTP 200 detail={rj.get('detail','')!r}")
        return (True, status, rj, None)

    # ── 결과 발행 (/action_result — GSP/ fake_hand_eye 계약) ──
    def _publish_result(self, req, x, y, ok, http_status, resp_json, err):
        slot = req["slot"]
        raw = req["raw"]
        out = {
            "step": raw.get("step"),
            "action": "pyramid",
            "color": raw.get("color"),
            "result": "success" if ok else "fail",
            # GSP.action_result_reflected() 가 stack(canonical 키)과 대조하므로
            # API slot(1l) 을 canonical(L1_left) 로 역매핑해 같이 싣는다.
            "target_slot": _API_TO_LLM_SLOT.get(slot),
            "slot": slot,
            "x": None if x is None else round(float(x), 4),
            "y": None if y is None else round(float(y), 4),
            "http_status": http_status,
            "detail": (resp_json.get("detail", "")
                       if isinstance(resp_json, dict) else ""),
            "error": err,
        }
        msg = String()
        msg.data = json.dumps(out, ensure_ascii=False)
        self.result_pub.publish(msg)
        self.get_logger().info(f"[result] {self.action_result_topic} ← {msg.data}")

    # ── pick 시퀀스 (run 루프에서 호출) ────────────
    def _process(self, req):
        self._busy = True
        slot = req["slot"]
        color = req["raw"].get("color")
        log = self.get_logger()
        log.info(f"=== pick 시퀀스 시작 (slot={slot}, color={color!r}) ===")
        x = y = None
        try:
            raw = req["raw"]
            if raw.get("x") is None or raw.get("y") is None:
                self._publish_result(
                    req, None, None, False, None, None, "move_result_missing_xy")
                return
            ref_xy = np.array([float(raw["x"]), float(raw["y"])], dtype=float)
            p = self._select_pick_xy(color, ref_xy)
            if p is None:
                self._publish_result(
                    req, None, None, False, None, None, "select_failed")
                return
            x, y = float(p[0]), float(p[1])

            # The skill server homes the arm after placing (clears the exo view)
            # before returning, so by the time this 200 arrives the scene is
            # clean — publishing /action_result here is safe for GSP's world wait.
            ok, status, rj, err = self._call_pyramid(x, y, slot)
            self._publish_result(req, x, y, ok, status, rj, err)
        except Exception as e:  # noqa: BLE001 - 어떤 예외든 result 로 보고
            log.error(f"[pick] 예외: {e}")
            self._publish_result(req, x, y, False, None, None, f"exception: {e}")
        finally:
            self._busy = False
            log.info(f"=== pick 시퀀스 종료 (slot={slot}) ===")

    # ── 메인 루프 ────────────────────────────────
    def run(self):
        log = self.get_logger()
        log.debug(f"[Init] 대기 중 — {self.move_result_topic} 에 유효 slot 들어오면 동작")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._pending is not None and not self._busy:
                req = self._pending
                self._pending = None
                self._process(req)


def main(args=None):
    rclpy.init(args=args)
    node = PickNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
