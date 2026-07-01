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
- [notes/original-obsidian-note.md](notes/original-obsidian-note.md) — 분실 전 원본 노트 원문

## 📊 현재 진행 (2026-06-30)
- ✅ ROS 2 Humble 환경 복구, git clone (`chaeseungyun/PNS_turtlebot3`)
- ✅ **Phase 1 코드/설정 재구성 완료** (behavior_server·inflation 0.22·particle_cloud·스핀 자동화) — 빌드/타입 검증 OK
- ⏳ **Phase 2 (로봇 연결 후)**: 재매핑(라이다 90° 반영), 카메라→costmap + camera TF, 실주행 검증
- 작업 위치: `~/turtlebot3_ws/src/PNS_turtlebot3` (이 VM은 임시 → **코드+문서 통합 repo**를 git로 새 PC 이전)
