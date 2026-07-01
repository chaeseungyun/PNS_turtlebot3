# 50 · Phase 2 문제 해결 절차 (로봇 연결 후)

작성: 2026-06-30 (로봇 연결·bringup 상태에서 실측 반영).
목표: 사용자 요구사항을 만족하는 **최종 단일 launch** 구축.
> 최종 동작 순서: **AMCL 수렴 → 카메라 실행 → 순회**, 한 launch 파일로.

## 0. 실측으로 확정된 사실 (2026-06-30)
- ✅ VM ↔ 로봇 ROS 그래프 연결됨 (ROS_DOMAIN_ID=30). `/scan` 10.6Hz, `/odom /imu /tf` 정상.
- ✅ **라이다 90° 확정**: `base_link→base_scan` rpy=(0,0,**90°**), xyz=(-0.064,0,0.122).
  각도 매핑: scan 0°=로봇 좌, 90°=후면, 270°=정면. → "라이다 오른쪽=정면, 왼쪽=후면" 일치.
- ✅ **좌측 팔 차폐 정상**: 실 `/scan`의 68-108°(41빔)가 <0.25m(매니퓰레이터 하부). 마스크 64-109°가
  이를 덮고, `/scan_filtered`에서 해당 구간 short 0개·inf 처리 확인. (**scan_arm_mask.py 그대로 유효**)
- ✅ **카메라 TF 이미 존재**: `base_link→camera_link` xyz=(0.073,-0.011,0.084) rpy=(0,0,0),
  `camera_link→camera_rgb_frame→camera_rgb_optical_frame` 체인도 bringup에서 발행 중.
  → 기본 Pi 카메라 프레임 기준이면 **static TF 추가 작업 불필요**.

## 1. 요구사항 (사용자, 2026-06-30)
| # | 요구 | 반영 방법 |
|---|---|---|
| R1 | 맵에서 **자동 초기 위치** 지정 후 **자동 회전으로 AMCL 수렴** | 오케스트레이터가 /initialpose 발행 + Nav2 Spin |
| R2 | 회전은 **한 방향 X, 좌우 왔다갔다(±)** | auto_patrol_node: ±π 번갈아 (이미 구현) |
| R3 | **AMCL 수렴 중엔 카메라 장애물이 costmap에 반영되면 안 됨** | 카메라 obstacle source를 수렴 완료까지 **게이트(비활성)** |
| R4 | 카메라로 얇은 책상다리·의자 바퀴 감지 → costmap 회피 | 카메라→obstacle 파이프라인 (설계 §4) |
| R5 | 일정 지점 좌표 **박아넣고 순회** | 재매핑 후 실좌표 waypoints.yaml |
| R6 | **최종 단일 launch**: AMCL 수렴 → 카메라 → 순회 순 | 오케스트레이터가 순서 강제 (§3) |
| R7 | 마지막에 디테일 Nav2 파라미터 튜닝 | 순회 검증 후 |

## 2. 왜 "게이트"가 필요한가 (R3 근거)
AMCL 수렴 전에는 로봇 위치 추정이 흔들리고, 이때 카메라가 만든 장애물이 costmap에 들어가면
- 잘못된 위치에 장애물이 찍혀 스핀 경로를 막고, 코스트맵-스캔 정합을 방해 → **수렴 실패**(경험칙).
따라서 **수렴 완료 신호 전까지 카메라 obstacle 입력을 차단**하고, 수렴 후 켠다.

## 3. 아키텍처 — 단일 launch에서 순서 강제
launch 타이밍(sleep)으로 순서를 맞추면 불안정. **오케스트레이터 노드가 ROS 상태로 순서를 제어**한다.
(현재 `auto_patrol_node.py`를 확장하거나 `mission_control` 노드 신설)

```
[한 launch]  bringup(로봇측, 별도) 
  ├─ scan_arm_mask.py           (/scan → /scan_filtered)      : 상시
  ├─ nav2 bringup               (amcl, costmaps, planner, controller, behavior_server)
  ├─ keepout_filter             (필터 서버 2종 + lifecycle)
  ├─ 카메라 드라이버             (이미지/포인트클라우드 발행)   : 상시(발행만)
  ├─ 카메라→obstacle 노드        (게이트됨: enabled=false로 시작)
  └─ mission_control(오케스트레이터)
        1) /initialpose 발행(home)
        2) behavior_server active 대기(get_state)
        3) ±π 좌우 스핀 반복 → /particle_cloud 수렴 확인   [카메라 obstacle OFF]
        4) 수렴 완료 → 카메라 obstacle source ENABLE
        5) 웨이포인트 순회 시작
```

### 게이트 구현 옵션 (R3)
- **(A·권장) costmap obstacle layer 토글**: 카메라용 observation source를 별도 ObstacleLayer로 두고,
  수렴 후 오케스트레이터가 `set_parameters`로 해당 layer `enabled`를 false→true.
- (B) 카메라→obstacle 노드가 "converged" 신호(토픽/서비스) 받기 전엔 costmap 토픽 발행 안 함.
- (C) 수렴 후 오케스트레이터가 카메라 obstacle 노드를 lifecycle activate.
→ A가 Nav2 표준 파라미터만으로 되어 가장 단순. (라이다 obstacle layer는 항상 ON, 카메라 layer만 게이트)

## 4. 카메라 → costmap 파이프라인 (R4) — 설계 (소실분 재작성)
소실돼 처음부터 설계. 카메라 종류에 따라 갈림 → **[미확인] 아래 질문 필요**.
- **깊이 카메라(RGB-D)면**: depth → `PointCloud2` → costmap **obstacle/voxel layer의 observation_source**
  (frame=camera_depth_optical_frame). 얇은 다리/바퀴도 포인트로 잡힘. 가장 곧바로 costmap 반영.
- **RGB 단독이면**: 바닥 세그멘테이션/객체검출 → 바닥평면 투영 → 가짜 LaserScan/PointCloud 합성 →
  costmap source. 구현 부담 큼. (현재 TF에 camera_rgb_optical_frame만 보임 → RGB일 가능성)
- 공통: 카메라 obstacle은 §3의 게이트 대상. `obstacle_max_range`·높이 필터로 천장/바닥 오탐 제거.

## 5. 단계별 실행 순서 (사용자 계획 순)
1. **[진행] 로봇 설정 마무리** — 연결·차폐 검증 완료 ✅. 남은 것: 카메라 노드 기동 확인.
2. **재매핑** — 본 주행맵 소실 → cartographer로 새 맵 작성(라이다 90° 이미 반영됨) → `~/pns-turtlebot3/maps/map.{pgm,yaml}` 저장. (my_cartographer 사용)
3. **AMCL 자동수렴 로직** — mission_control 1)~3) 구현·검증 (카메라 OFF 상태). R1·R2.
4. **카메라 연동** — §4 파이프라인 + 게이트(R3). 카메라만 켜 obstacle이 costmap에 뜨는지 확인.
5. **장애물 회피 검증** — 얇은 다리/의자 바퀴 앞에서 회피하는지 실주행.
6. **좌표 박고 순회** — 맵상 순찰 지점 실좌표를 waypoints.yaml에 기입 → 순회.
7. **단일 launch 통합** — §3 순서로 묶기. R6.
8. **Nav2 디테일 튜닝** — inflation/critics/속도 등 최종 조정. R7.

## 4b. 카메라 실측 (2026-07-01 확정)
- 토픽: **`/image_raw`(+`/compressed`)**, `/camera_info`. 해상도 **640×480**. **RGB 단독**.
- ⚠️ **미캘리브레이션**: `/camera_info`의 K(fx,fy,cx,cy)=0, D 비어있음. IPM 정확도 위해 **캘리브레이션 필요**
  (`ros2 run camera_calibration cameracalibrator` + 체커보드). 임시로 fx≈fy≈500(640×480 추정) 사용 가능.
- **기하(합성, base_footprint 기준)**: 카메라 지면 높이 **h≈0.103m**, 전방 x≈0.076m, **수평 정면(틸트 0)**.
  광학프레임 회전 표준(camera_rgb_frame→optical rpy=-90,0,-90).
- **수평 카메라 평지 IPM**: 지면점 깊이 `Z = h·fy/(v−cy)` (v=픽셀행, cy=주점). 수평선(v=cy) 위는 지면 아님.
- 이전 구현 방식(사용자): **바닥색 대비로 장애물 판정 + 카메라 높이·각도로 거리 + HSV(채도·명도)로 햇빛 대응.**
  → 재작성: HSV 바닥 모델 학습(하단 ROI를 바닥으로 가정) → 픽셀별 바닥/비바닥 → 비바닥 최하단 경계를 IPM으로 지면 투영 → `PointCloud2`(base_footprint) → costmap obstacle source(게이트 대상).
- 현 상태 주의: 캡처 프레임상 **로봇이 책상 위**(바닥 안 보임)라 바닥검출 실검증은 로봇을 바닥에 놓아야 가능.

## 6. 미확정/질문
- ❓ **카메라 종류**: RGB-D(depth)인가 RGB 단독인가? (파이프라인 §4가 갈림). 모델명?
- ❓ 카메라 노드는 무엇으로 띄우나(v4l2_camera / realsense / usb_cam / raspicam)? 토픽명?
- ❓ 재매핑 시 맵 저장 위치를 repo 내(`my_nav2/map`)로 커밋할지, 워크스페이스 밖(~/maps)로 둘지.
