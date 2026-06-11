# Pick point 산출 — mask 무게중심 → 원(rim) 중심

`upright_cup_pose_node` 가 컵의 어느 픽셀을 pick 좌표로 보낼지 결정하는 로직 설명과
정지영상 검증 결과.

## 문제

똑바로 선(upright) 컵을 hand-eye 카메라로 위에서 보면 윗면 원(rim)이 보인다.
이상적으로는 그 원의 중심을 pick 하면 된다. 그런데 YOLO-seg mask 에 컵 **옆면이
같이 잡혀 mask 가 아래로 길쭉**해지는 경우가 잦다. 이때 기존처럼 mask 최대 contour
의 **moments 무게중심(centroid)** 을 pick point 로 쓰면, 무게중심이 옆면 쪽으로
끌려 내려가 **원 중심에서 벗어나** pick 이 부정확해진다.

## 해결 — mask 에서 "원" 중심을 다시 잡는다

`pick_point_method` 파라미터로 선택한다 (기본 `inscribed`).

| 값 | 방식 | 특징 |
|---|---|---|
| `top_hole` | 윗면 **도넛 홀(어두운 중앙 구멍)** 중심 | 컵 입구/관통홀의 진짜 원 중심을 잡는다. seg mask 는 채워진 실루엣이라 이미지에서 직접 검출. 실패 시 `inscribed` 폴백. (강건성 설계는 아래 참고) |
| `inscribed` (기본) | distance transform 최댓값 위치 = **가장 큰 내접원 중심** | 길쭉한 옆면 꼬리를 무시하고 둥근 윗부분 중심을 잡음. 파라미터 튜닝 불필요, 가장 강건. |
| `hough` | 이미지에서 `HoughCircles` 로 rim 원을 직접 검출 | '원 검출'에 가장 직접적이나 조명/로고 텍스처에 민감, 반경 튜닝 필요. 실패 시 `inscribed` 폴백. |
| `centroid` | 기존 moments 무게중심 | 변경 없음 — 비교/폴백용. |

`hough` 튜닝값: `hough_dp`, `hough_param1`, `hough_param2`,
`hough_min_radius_ratio`, `hough_max_radius_ratio` (반경 비율은 contour bbox 짧은 변 기준).

### `top_hole` — 윗면 도넛 홀 검출과 조명 강건성

seg mask 만으로는 도넛을 못 뽑는다(채워진 실루엣). 그래서 mask 내부 **이미지**에서
어두운 중앙 구멍을 찾는데, 단순 밝기 임계는 조명에 약하므로 다음을 적용했다.

1. **탐색 범위(`×top_hole_face_ratio`, 기본 2.5)** — 내접원 중심 기준 디스크 안의
   어두운 영역을 본다. **기운 컵은 입구 중심이 내접원(몸통쪽 치우침)에서 멀어**, 좁게
   잡으면(≈1.0) 입구가 후보에서 빠지고 볼트구멍만 남는다 → 넉넉히(≥2) 둬 입구를 포함.
2. **Otsu 자동 임계** — 절대 밝기가 아니라 림(밝음)/홀(어두움) 분포의 골을 찾아 갈라
   조명 변화에 적응. Otsu 가 비정상으로 높으면 `top_hole_dark_percentile` 상한으로 가드.
3. **면적 지배 선택** — pick 대상(컵 입구/center 구멍)은 rim 안에서 **가장 큰** 어두운
   영역이고 볼트구멍은 작다. 점수 `= 면적 × (1 − penalty·(dist/face_r)²)` 로 면적을
   지배항, 중심성을 가장자리 약한 감점으로만 둔다(`top_hole_centrality_penalty`).
   원형도(`top_hole_min_circularity`)·면적비(`top_hole_min/max_area_frac`)는 hard gate.
   → 볼트구멍 오선택이 영상 검증에서 **10% → ~4%** 로 감소(나머지는 대부분 자세상
   입구가 작게 보이는 정상 선택).
4. **중심은 무게중심(moments)** — minEnclosingCircle 중심보다 외곽 노이즈에 덜 흔들림.
5. **폴백 체인** `top_hole → inscribed → centroid` — 홀이 안 보이는 가파른 각도·가림에선
   엉뚱한 값 대신 내접원으로 안전 복귀.

튜닝값: `top_hole_face_ratio`(0.95), `top_hole_min_circularity`(0.45),
`top_hole_dark_percentile`(35), `top_hole_min_area_frac`(0.01),
`top_hole_max_area_frac`(0.7).

### 합성 mask 수치 검증

원(중심 `(100,100)`, r40) 아래에 좁고 긴 옆면 꼬리를 붙인 mask 로 확인:

| 방식 | 결과 | 비고 |
|---|---|---|
| centroid(기존) | `(100, 145)` | 옆면 꼬리에 끌려 내려감 |
| inscribed(신규) | `(100, 100)`, r≈39.5 | **정확한 원 중심** |

## debug 영상 (`/upright_cup/debug_image`)

컵마다 세 가지를 함께 그려 눈으로 비교할 수 있다.

- 🟢 **초록 원** — 검출된 원(내접원/hough)의 둘레
- 🔴 **빨강 점** — 최종 pick point (원 중심)
- ⚪ **회색 점** — 기존 무게중심(centroid)
- 🟡 노란 윤곽 — seg mask contour

> 정지영상 검증에선, 실제 hand-eye 가중치(`upright-cup` 클래스)로 컵 영상에 노드와
> **동일한 검출 + 내접원 + debug 그리기** 로직을 적용했을 때, 옆면이 아래로 잡힌 컵은
> 회색(centroid)이 내려가 있어도 빨강(pick)이 윗면 원 쪽으로 올라옴을 확인했다.
> (depth/FK/로봇 없이 2D 검출만으로 산출되는 debug 부분만 재현 — 라이브 카메라+로봇
> end-to-end 검증은 실제 리그에서 별도 필요.)

## 중복 검출 제거 (`dedup_min_dist_px`)

YOLO NMS(`iou`)가 못 거른 겹친 중복 검출(같은 컵이 IoU 임계 바로 아래로 두 번
잡히는 경우)을 정리한다. **pick point 가 `dedup_min_dist_px`(기본 25px) 안인 같은
클래스 검출은 conf 가 높은 것만 남긴다.** `0` 이하면 비활성.

> 예: 한 컵이 `(546,406) conf=0.66` / `(546,405) conf=0.65` 로 0.7px 차이로 이중
> 검출되던 것을 conf 높은 하나로 합쳐, 검출 수 5→4 로 정정됨.

## 시간 평활 / 트래킹 (`enable_temporal_smoothing`)

영상에서 보면 단일 프레임 pick 은 흔들리고, 특히 `top_hole` 은 **구멍이 여러 개인
부품(center + 볼트홀)에서 ~10% 빈도로 볼트홀을 오선택**한다(영상 627프레임 검증).
이를 줄이기 위해 검출 컵을 **base_link 공간**에서 프레임 간 추적해 평활한다.

- **base 공간 평활**: hand-eye 카메라가 움직여도 정지 컵은 base_link 에서 고정이라,
  카메라 모션과 무관하게 튐만 걸러진다(이미지 공간 평활이면 모션에 끌려감).
- **매칭→EMA→outlier 게이트**: 최근접 트랙(`track_match_dist`)에 붙여 `smoothing_alpha`
  로 EMA. 측정이 평활값에서 `track_outlier_dist`(기본 4cm) 이상 튀면 **단발 outlier 로
  보고 무시(평활값 유지)**, `track_reacquire_frames` 연속되면 컵이 실제 이동한 것으로
  재획득. `track_match_dist`(기본 8cm)는 컵 간격(≈10cm)보다 작고 `track_outlier_dist`
  보다 커야 한다(outlier 가 트랙에 붙어 게이트로 걸러지도록).
- 트랙 id 는 안정적으로 유지돼 마커 id(`box_top`/`box_labels`)로도 쓰인다.

> 단위검증: 정지 컵에 5cm outlier 2프레임을 주입해도 published 좌표가 참값 ±2mm 로
> 유지(게이트가 차단), 일반 잡음(±5mm)은 ~2mm 로 평활됨.

**권장 조합(robust)**: `pick_point_method:=inscribed`(부품 축 중심, 구멍 무시 →
볼트홀 오선택 원천 제거) + 시간 평활(잔여 jitter·outlier 제거). `top_hole` 은 입구가
하나로 깨끗한 컵에서만 권장.

### 재현 / 실환경 확인

```bash
# 실제 리그 (카메라 + 로봇)
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m0609 host:=192.168.1.100
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true align_depth.enable:=true
ros2 launch hand_pick upright_cup_pose.launch.py            # pick_point_method:=inscribed (기본)
ros2 run rqt_image_view rqt_image_view /upright_cup/debug_image
```

내접원이 맞지 않으면 `pick_point_method:=hough` 로 바꿔 비교한다.
