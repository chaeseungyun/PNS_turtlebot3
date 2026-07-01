# 60 · 네트워크 제약과 소프트웨어 우회 (2026-07-01)

## 증상
개발 PC(VMware VM, Wi-Fi 브리지)에서 로봇의 `/scan`·`/image`(큰 토픽)가 **전혀 안 옴**.
`/imu /odom /battery` 등 작은 토픽은 정상.

## 근본 원인 (실측으로 확정 — DDS 문제 아님)
- `ping -s 1400`(1428B, MTU 이하 = 1패킷): **0% 손실**
- `ping -s 2000`(2028B, MTU 초과 = IP 단편화): **100% 손실 (양방향)**
- 재부팅·RMW(Fast/Cyclone)·브리지 설정 무관하게 동일.
- 로봇 Wi-Fi는 정상(−49dBm, 5GHz, 433Mbps). 로봇은 `/scan` 로컬 10Hz 발행.

→ **호스트 노트북의 Wi-Fi 어댑터 + VMware 브리지가 "MTU를 넘는 단편화 IP 패킷"을 100% 버린다.**
  - `/scan`(~3KB: ranges 400 + intensities 400)·`/image`(60KB)는 단편화되어 소실.
  - 작은(1패킷) 메시지는 통과(단, 10Hz 부하에선 단일 패킷도 ~45% 손실 → reliable로 복구).
- **이전 노트북에선 되던 이유**: 물리적으로 다른 노트북 = 다른 Wi-Fi 칩셋/드라이버. 그 어댑터는
  브리지에서 단편화를 정상 처리했음. VM 소프트웨어(VMware Workstation Pro)는 동일하나 하드웨어가 다름.
  (이번 세션 초반 10.6Hz는 브리지가 깨지기 전 상태였을 가능성 — 단, 재부팅으로도 복구 안 돼 하드웨어 한계로 결론)

## 해결책
### A. 근본책(권장): 노트북 유선 랜 연결
호스트를 랜선/USB-이더넷으로 연결 → Wi-Fi 어댑터를 안 거침 → 단편화 정상 → 원본 데이터 그대로 PC.
"무거운 처리 = PC" 구조를 그대로 쓸 수 있음.

### B. 소프트웨어 우회 (지금 채택, Wi-Fi 유지)
**"큰 원본은 로봇 로컬에서만, PC로는 1패킷 크기의 가공 데이터만"**
- **`/scan`**: 로봇에서 `scan_relay.py` 실행 → `/scan_relay`
  - 다운샘플(step=2: 400→200pts) + **intensities 제거** + **RELIABLE** → ~850B(1패킷) → **PC 10.1Hz 검증 완료**.
  - Nav2/AMCL 의 scan_topic 을 `/scan_relay` 로 설정해 사용. (AMCL은 200pts로 충분)
- **`/image`**: 로봇에서 `camera_obstacle_node.py` 실행 → 작은 `/camera_obstacles`(PointCloud)만 PC로.

```
[로봇]  bringup ─ /scan(400,best_effort,로컬) ─ scan_relay ─▶ /scan_relay(200,reliable,1패킷) ─Wi-Fi─▶ [PC] 10Hz
[로봇]  v4l2 ─ /image(로컬) ─ camera_obstacle_node ─▶ /camera_obstacles(작음) ─Wi-Fi─▶ [PC]
```

## 검증 기록
| 대상 | 원본(그대로) | 우회 |
|---|---|---|
| /scan | 0 Hz | **/scan_relay 10.1Hz, 200pts** ✅ |
| 큰 best_effort 5KB | 0 Hz | (다운샘플로 대체) |
| /image 60KB | 0 Hz | 로봇 로컬 처리 → /camera_obstacles |

## 남은 일 / TODO
- `scan_relay`에 좌측 팔 차폐(64~109° 마스킹) 통합 → 출력 `/scan_filtered` (Nav2 설정과 직결).
- 로봇 bringup에 relay + camera 노드 자동시작 등록.
- 유선 연결이 가능해지면 A로 전환(우회 불필요).
