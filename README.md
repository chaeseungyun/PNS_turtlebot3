# pns-turtlebot3 — TurtleBot3 자율주행(AMR) 복구 프로젝트

> 노트북 분실로 인한 **작업 환경 복구** 진행 중. 이 폴더는 복구·재구성 과정의 단일 기준 문서함이다.
> 최초 작성: 2026-06-30 (복구 1일차)

## 🎯 목표
- TurtleBot3 **Waffle Pi** + **ROS 2 Humble**로 실내 자율주행(AMR).
- 분산 구성: **로봇(라즈베리파이)** = bringup(센서·구동)만 / **개발 PC** = SLAM · Nav2 · RViz.
- 1순위: 의자·책상 등 장애물에 부딪히지 않고 주행 → 이후 웨이포인트 자동 순찰.
- 매니퓰레이터 팔은 일시 제거, AMR 주행에 집중. (단, 팔 하부 마운트는 물리적으로 남아 라이다에 잡힘 → 차폐 처리)

## 📁 저장소 구조
> 코드와 복구 문서를 **한 저장소로 통합** — clone 한 번이면 코드+문서가 함께 이동한다.
- `src/my_nav2/` — Nav2 파라미터·launch·waypoint·keepout·스크립트 (`config/`, `param/`, `map/`)
- `src/my_cartographer/` — SLAM(Cartographer) 설정·launch·rviz
- `docs/` — 복구 상태, 시스템 구성, 의사결정 기록 등 정리 문서
- `notes/` — 원본 노트(Obsidian) 백업, 단편 메모

## 🔑 핵심 문서
- [docs/00-recovery-status.md](docs/00-recovery-status.md) — **지금 무엇이 남아있고 무엇이 사라졌나** (가장 먼저 볼 것)
- [docs/10-hardware-facts.md](docs/10-hardware-facts.md) — 로봇/센서 물리 구성(라이다 90° 등) 복구 사실
- [docs/30-gap-analysis.md](docs/30-gap-analysis.md) — git 베이스라인 vs 목표 상태 격차
- [docs/40-reconstruction-log.md](docs/40-reconstruction-log.md) — Phase 1 재구성 내역·검증
- [docs/50-phase2-procedure.md](docs/50-phase2-procedure.md) — Phase 2 절차(요구사항 R1~R7) + 라이다90°·차폐·카메라 실측
- [docs/60-network-workaround.md](docs/60-network-workaround.md) — **VM Wi-Fi 단편화 손실 + scan_relay 우회**
- [notes/original-obsidian-note.md](notes/original-obsidian-note.md) — 분실 전 원본 노트 원문

## 📊 현재 진행 (2026-07-01)
- ✅ ROS 2 Humble 환경 복구, git clone, Phase 1 코드/설정 재구성(빌드/타입 검증 OK)
- ✅ **로봇 실측**: 라이다 90°(base_scan yaw=90) 확정, 좌측 팔 차폐 정상, 카메라 TF 존재
- ✅ **카메라→costmap 노드 재작성**(`camera_obstacle_node.py`): 바닥 HSV+IPM, 로봇 로컬 실행, 근거리 검출 실용 수준
- ✅ **네트워크 병목 해결**: VM Wi-Fi 브리지가 단편화(큰) 패킷 100% 손실 → `scan_relay.py`(다운샘플+reliable)로 `/scan` PC 10Hz 확보. [[60-network-workaround]]
- ⏳ 남은 것: scan_relay에 팔 차폐 통합(/scan_filtered), 노드 자동시작, AMCL 자동수렴→순회, 카메라 캘리브레이션
- 작업 위치: `~/turtlebot3_ws/src/PNS_turtlebot3` (코드+문서 통합 repo, git로 새 PC 이전 가능)
