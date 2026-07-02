# 60 · 네트워크 제약과 소프트웨어 우회 (2026-07-01)

> **⚠️ 업데이트 (2026-07-02, 로봇 재부팅 후): 아래 "0Hz" 전제가 더 이상 성립하지 않음.**
> 재부팅 후 실측에서 **원본 `/image_raw`가 PC에 우회 없이 직접 수신됨**.
> - `/image_raw`: 640×480 `rgb8`, ~900KB/frame, **14.3Hz 안정 수신**(std dev 0.004s), QoS RELIABLE, 발행자 `v4l2_camera`(로봇).
> - PC에 로컬 카메라 없음 확정(`/dev/video*` 없음, v4l2 프로세스 없음) → **로봇 발행 원본이 네트워크를 그대로 건너옴**.
> - `/image_raw/compressed`: discovery엔 뜨나 실제 0Hz(발행 안 됨, image_transport republish 미실행).
> - `/scan`도 동일하게 우회 없이 직접 수신됨(같은 날 앞선 실측: 10Hz 안정).
>
> **의미**: "이미지 처리 = 로봇 로컬" 강제 제약이 풀림 → **이미지 로직을 PC로 옮기는 구조가 지금 가능**.
> `camera_obstacle_node.py`를 로봇 대신 PC에서 `/image_raw` 구독해 실행하면 됨.
>
> **단, 재현성 미확정**: 이전엔 "재부팅 무관 하드웨어 한계"로 결론냈던 문제라 재부팅 한 번으로
> 풀렸다고 단정 불가(AP 재연결/채널 변경 등 다른 요인 가능). 또 같은 날 더 앞선 측정에선
> `/image_raw`가 ~7Hz·최대 갭 0.84s로 불안정했던 기록도 있음 → **실주행 부하 상황에서 재확인 권장.**
> 유선(A안)으로 전환하면 이 불확실성 자체가 사라짐.

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
| 대상 | 07-01 (브리지 고장 시) | 07-02 (로봇 재부팅 후, 우회 없이 직접) |
|---|---|---|
| /scan | 0 Hz | **10Hz 안정 직접 수신** ✅ |
| 큰 best_effort 5KB | 0 Hz | (미재측정) |
| /image_raw (~900KB/frame, 640×480 rgb8) | 0 Hz | **14.3Hz 안정 직접 수신**(std dev 0.004s) ✅ ／ 단 같은 날 앞선 측정에선 ~7Hz 불안정 기록도 있음 |

> 07-01 표는 "브리지가 단편화 패킷을 100% 드랍하던 상태"의 기록. 07-02 재부팅 후 이 증상이
> 해소된 것으로 보이며 원본 토픽들이 직접 수신됨(재현성은 위 상단 업데이트 참조).

## 남은 일 / TODO
- **원본 직접 수신의 재현성/안정성 재확인**: 실주행 부하(Nav2+주행 중)에서 `/image_raw`·`/scan` Hz 재측정.
  안정적이면 scan_relay 우회 자체가 불필요해짐.
- **이미지 로직 PC 이전**(원본 직접 수신이 되므로 지금 가능): `camera_obstacle_node.py`를 로봇 대신
  PC에서 `/image_raw` 구독하게 실행 → costmap observation_source 등록.
- (우회 유지 시) `scan_relay`에 좌측 팔 차폐(64~109° 마스킹) 통합 → 출력 `/scan_filtered`.
- 로봇 bringup에 relay + camera 노드 자동시작 등록.
- 유선 연결이 가능해지면 A로 전환(우회 불필요 + 위 재현성 불확실성 제거).
