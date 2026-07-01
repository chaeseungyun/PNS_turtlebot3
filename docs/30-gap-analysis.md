# 30 · 격차 분석 (Baseline vs 목표)

- **Baseline** = git `chaeseungyun/PNS_turtlebot3` @ `407174e` (2026-06-08, 커밋 1개)
- **목표 상태** = 원본 노트의 6/9~6/10 의사결정 + 카메라 연동(소실)
- 결론: 이 git은 **모든 핵심 튜닝 직전의 베이스라인**. 아래 격차를 메우는 게 Phase 1.

## 가져온 자산 (git에 존재, 재사용 가능)
- 패키지 골격: `my_nav2`, `my_cartographer` (CMakeLists, package.xml, launch, rviz)
- `scripts/scan_arm_mask.py` — 좌측 자기차폐 마스킹 노드 (band-pass /scan→/scan_filtered)
- `scripts/auto_patrol_node.py` — 초기포즈+수렴대기+웨이포인트 순회 (단, 버그 있는 옛 버전)
- `scripts/make_keepout_mask.py` — keepout 마스크 생성
- `param/humble/waffle_pi.yaml` — Nav2 파라미터 (옛 값)
- `map/keepout_mask.{pgm,yaml}` — keepout 마스크 (자체완결, OK)
- `map/tune_groupA.{pgm,yaml}` — 튜닝용 맵 (yaml은 `/home/pns/maps/...` 절대경로 참조)
- launch: `my_navigation2`, `keepout_filter`, `auto_patrol` + cartographer launch/lua

## 소실 (git에도 없음)
- ❌ **본 주행 맵 `map.yaml`/pgm** — 정식 위치가 `~/maps/`(gitignore)라 통째 소실 → **재매핑 필요**(어차피 라이다 90° 반영 예정)
- ❌ **카메라→costmap 연동 일체** — 카메라 노드/observation source 없음 (6/8 이후 작업)
- ❌ 카메라 TF (camera_link→base_link)

## 메워야 할 격차 (우선순위)

### P1. 코드/설정 재구성 (로봇 없이 가능, 노트에 근거 있음)
| # | 파일 | 현재(베이스라인) | 목표 | 근거(노트) |
|---|---|---|---|---|
| 1 | `auto_patrol_node.py` | `/particle_cloud`를 `PoseArray`+기본QoS로 구독 | `nav2_msgs/ParticleCloud` + **best_effort QoS** | "한 번도 시작 못하던 버그: /particle_cloud 타입·QoS 불일치" |
| 2 | `auto_patrol_node.py` | 스핀 자동화 없음(수동 수렴 대기만) | 초기포즈 후 **Nav2 Spin ±π** 자동 실행해 AMCL 수렴 | "위치추정 스핀 자동화(±π)" |
| 3 | `auto_patrol_node.py` | 스핀 없음 | **behavior_server `get_state`로 active 확인 후** goal 전송(거부 회피) | 6/10 "active 전 goal REJECT" |
| 4 | `waffle_pi.yaml` L344 | `recoveries_server` 섹션 | **`behavior_server`** + 플러그인 `nav2_behaviors/*` + `behavior_plugins` | 6/9 "섹션명 교정, 안그럼 통째 무시" |
| 5 | `waffle_pi.yaml` L361 | spin `max_rotational_vel: 1.0` | **`0.4`** | 6/9 "스핀속도 1.0→0.4" |
| 6 | `waffle_pi.yaml` L190·288 | `inflation_radius: 0.25` | **`0.22`** (inscribed 하한 0.17) | 6/10 "inflation 0.45→0.22, free 68→79%" |
| 7 | `waypoints.yaml` | `home (0,0,0)` + 예시 waypoint A/B/C | home `(0.115,-0.075,0)` ⚠️맵 의존 / 실제 waypoint 미정 | 6/10 home 실측 |

### P2. 로봇 연결 후 (Phase 2)
- 라이다 90° 보정 위치/정합성 검증 → **재매핑**으로 새 `map.yaml` 생성
- 카메라 모델·장착위치 실측 → **camera_link→base_link static TF**
- 카메라 영상→장애물 검출→**costmap observation source** 재구현 (얇은 책상다리/의자 바퀴)
- home/waypoint 새 맵 기준으로 재확정 + 순회 검증

## ⚠️ 미해결 모순 (사용자 확인 필요)
**라이다 방향 충돌:**
- `scan_arm_mask.py`(6/8 코드, live verify): "scan 0°=robot **front**, base_link→base_scan yaw=0", 좌측 차폐 띠 **64~109°**
- 사용자 기억(6/30): "라이다 90° 회전, 라이다 오른쪽=정면, 왼쪽=후면"
- → 6/8 이후 90°를 발견/보정한 것으로 추정. **보정을 어디에 넣었는지**(bringup 라이다 드라이버 / URDF base_scan / 마스크 각도)에 따라 마스크 띠 64~109°가 유효한지 달라짐. launch엔 라이다 static TF 없음.
