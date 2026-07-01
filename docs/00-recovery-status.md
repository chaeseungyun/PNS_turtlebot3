# 00 · 복구 상태 (Recovery Status)

작성: 2026-06-30 / 갱신 시 날짜 추가

## A. 검증된 현재 머신 상태 (이 VM에서 직접 확인함)
| 항목 | 상태 |
| --- | --- |
| OS | Ubuntu 22.04.5 LTS / x86_64 (가상머신, `pns-virtual-machine`) |
| ROS 2 | **Humble 설치 완료** (`/opt/ros/humble`) — 이번 복구 세션에서 새로 설치 |
| 의존 패키지 | gazebo_ros_pkgs, cartographer(+ros), navigation2, nav2_bringup **설치됨** |
| TurtleBot3 | dynamixel-sdk, turtlebot3-msgs, turtlebot3, turtlebot3-simulations **설치됨(apt)** |
| bashrc | `ROS_DOMAIN_ID=30`, `TURTLEBOT3_MODEL=burger`(⚠️ 실제는 waffle_pi로 바꿔야 함), `LDS_MODEL=LDS-02`, humble source |
| Gazebo 시뮬 | 동작 확인됨(turtlebot3_world + burger 스폰, VM이라 `LIBGL_ALWAYS_SOFTWARE=1` 필요) |

## B. 사라진 것 (이 머신에 없음 — 분실 노트북에 있던 것)
- ❌ `~/turtlebot3_ws/src/my_nav2` — **핵심 Nav2 패키지(전부 소실)**
  - Nav2 파라미터(yaml), launch, 위치추정 자동 스핀 노드, **좌측 차폐(keepout/마스킹) 노드**, 카메라→costmap 연동
- ❌ `~/robot_project` — 프로젝트 문서
- ❌ SLAM 맵 파일(.pgm/.yaml), keepout 마스크 이미지
- ❌ git 저장소 흔적 없음, 로컬 맵/yaml 흔적 없음

## C. 백업 조사 결과
- Google Drive: 사업용 자료(차단기 로봇 등)만 존재. **my_nav2 코드/맵 백업 미발견.**
- → 다른 백업처(git 원격, 로봇 SD카드, USB/타 PC) 확인 필요 → **사용자 질문 중**

## D. 복구 자산 (가지고 있는 것)
- ✅ 분실 전 **Obsidian 프로젝트 노트** (의사결정·튜닝값 상당수 기록) → `notes/original-obsidian-note.md`
- ✅ 사용자 기억(라이다 90°, 좌측 차폐, 카메라 연동, 로봇 22cm 등) → `docs/10-hardware-facts.md`

## E. 확정된 복구 계획 (2026-06-30 사용자 확인)
사용자 답변:
1. 백업: **git 원격에 my_nav2 존재 — 단 "오래된 버전"** ✅ (clone 가능)
2. 실물 로봇: 있지만 **지금 연결 안 됨** → 당장은 로봇 없이 가능한 작업
3. 이 VM: **임시** 환경, 나중에 실제 개발 PC로 이전 → 작업은 git로 포터블 유지
4. 우선순위: **코드/설정 재구성 먼저**

### → 복구 전략
- **Phase 1 (지금): 코드 복원·재구성**
  1. git 원격에서 my_nav2 **clone** (오래된 버전) → `~/turtlebot3_ws/src/my_nav2`
  2. clone 내용 vs 노트의 최종 상태(decisions) **diff** → 무엇이 빠졌는지 식별
  3. 노트 기록대로 **끌어올리기**: behavior_server 섹션명, 스핀 자동화+active 대기, inflation 0.22, home 좌표, 좌측 차폐 노드, 카메라→costmap, 라이다 90° 보정
  4. `colcon build` 로 빌드 검증(로봇 없이 가능한 범위)
- **Phase 2 (로봇 연결 후): 실로봇 검증** — bringup, 재매핑(라이다 교정 반영), Nav2 주행, 카메라 TF 실측
- **이식성**: 모든 작업물 git 커밋 → 새 PC에서 clone으로 이전

### 즉시 필요한 입력
- **git 원격 저장소 URL** (또는 GitHub 사용자/repo 이름). 비공개면 인증수단(PAT 또는 SSH 키) 필요.
