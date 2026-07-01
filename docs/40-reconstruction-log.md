# 40 · 재구성 로그 (Phase 1)

작성: 2026-06-30. 대상 워크스페이스: `~/turtlebot3_ws/src/PNS_turtlebot3` (git clone).
방침: **노트대로 재구성 + 베이스라인 값 주석 보존**. 로봇 미연결 → 실주행 검증은 Phase 2.

## 변경 파일 3개

### 1) `src/my_nav2/param/humble/waffle_pi.yaml`
- `recoveries_server` → **`behavior_server`** 섹션명 교정 (Humble 노드명).
  - 플러그인 네임스페이스 `nav2_recoveries/*` → `nav2_behaviors/*`, `recovery_plugins` → `behavior_plugins`.
  - 근거: Humble nav2_bringup 라이프사이클 매니저가 관리하는 노드명이 `behavior_server`라
    옛 이름 섹션은 **bring-up 자체가 안 됨** → 스핀/백업/속도 설정 전부 무시되던 원인.
- spin `max_rotational_vel`: **1.0 → 0.4** (노트 06-09, 과회전·충돌 방지).
- `inflation_radius`: **0.25 → 0.22** (로컬·글로벌 코스트맵 둘 다, 노트 06-10).
- 베이스라인 원본 값은 모두 주석으로 보존.

### 2) `src/my_nav2/scripts/auto_patrol_node.py` (대폭 재구성)
- **[수정1] `/particle_cloud` 버그**: `geometry_msgs/PoseArray` + 기본 QoS(10)
  → **`nav2_msgs/ParticleCloud` + best_effort(`qos_profile_sensor_data`)**.
  - 콜백 미수신으로 "순찰이 한 번도 시작 못하던" 근본 버그. 수렴계산도 `.particles[*].pose`로.
- **[수정2] 위치추정 자동 스핀**: 초기포즈 발행 후 **Nav2 `Spin` 액션으로 ±π 왕복** 회전,
  매 회전 끝에 수렴 확인. 미수렴이면 반대방향, 최대 8회.
- **[수정3] behavior_server active 대기**: 스핀 goal 전 `/behavior_server/get_state`를
  폴링해 `PRIMARY_STATE_ACTIVE(3)` 확인 후 전송 (active 이전 goal REJECT 회피).
- 웨이포인트 순회 로직은 베이스라인 유지.

### 3) `src/my_nav2/package.xml`
- `lifecycle_msgs`(신규 사용), `sensor_msgs`(scan_arm_mask가 쓰는데 누락돼 있던 것) 의존성 추가.

## 검증 (로봇 없이 가능한 범위)
- ✅ `python3 -m py_compile` 두 스크립트 문법 OK
- ✅ `colcon build --packages-select my_nav2 my_cartographer` 성공 (exit 0)
- ✅ 메시지 타입/필드 실측 검증 (이 Humble 설치 기준):
  - `nav2_msgs/ParticleCloud.particles : sequence<nav2_msgs/Particle>`, `Particle.pose/weight`
  - `nav2_msgs/action/Spin.Goal.target_yaw : float`
  - `lifecycle_msgs/srv/GetState`, sensor QoS import OK

## 미검증 (Phase 2, 로봇 필요)
- 실제 AMCL 수렴이 ±π 스핀으로 일어나는지, 스핀 속도 0.4가 적절한지
- inflation 0.22에서 실제 통행성 (노트 "free 68→79%" 재현)
- behavior_server active 폴링 타이밍, /spin 서버 기동 순서

## 손대지 않은 것 (의도적)
- `scan_arm_mask.py` 마스크 띠 64~109° — **라이다 90° 보정 위치 불명**(사용자 기억 안 남)
  → Phase 2에서 /scan 실측 후 결정. [[30-gap-analysis]] ⚠️ 참고.
- `waypoints.yaml` home (0,0,0)·예시 waypoint — 재매핑으로 좌표계 바뀌므로 보류.
