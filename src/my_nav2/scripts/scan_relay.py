#!/usr/bin/env python3
"""
Scan Relay — 로봇에서 실행. 큰 /scan 을 '1 UDP 패킷' 크기로 줄여 reliable 로 재발행.

[왜 필요한가] 2026-07-01 실측:
  개발 PC(VMware VM, Wi-Fi 브리지)의 호스트 노트북 Wi-Fi 어댑터가 **MTU(1500B)를 넘는
  단편화 IP 패킷을 100% 버린다**(ping -s 2000 양방향 100% 손실, 재부팅·RMW 무관).
  → 원본 /scan(~3KB, ranges+intensities)은 여러 패킷으로 단편화되어 PC에 아예 안 옴.
  작은(1패킷) 메시지는 통과하므로, 로봇에서 다운샘플+intensities 제거로 1패킷에 담아 보낸다.

[동작]
  구독 /scan (best_effort, 로컬)  →  발행 /scan_relay (RELIABLE, 다운샘플, intensities 제거)
  - STEP 배수로 포인트 감솎음(기본 2: 400→200pts ≈ 850B → 1패킷)
  - intensities 제거(AMCL/Nav2 불필요) — 이게 없으면 2배 커져 단편화됨
  - RELIABLE QoS: 1패킷도 Wi-Fi 부하에서 ~45% 손실 → 재전송으로 10Hz 복구

  검증: PC 수신 /scan_relay = 10.1Hz, 200pts (원본 /scan은 0Hz).

[배포] 로봇에서:
  ros2 run my_nav2 scan_relay.py       (또는 python3 scan_relay.py)
[사용] PC의 Nav2/AMCL 이 scan_topic 을 /scan_relay 로 구독하게 설정.

TODO: 좌측 팔 차폐(scan_arm_mask 64~109°)를 이 노드에 통합해 /scan_filtered 로 출력하면
      Nav2 설정(scan_topic: scan_filtered)과 바로 연결됨. (지금은 다운샘플만)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy, HistoryPolicy)
from sensor_msgs.msg import LaserScan


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')
        self.step = self.declare_parameter('step', 2).value
        self.drop_intensities = self.declare_parameter('drop_intensities', True).value
        in_topic = self.declare_parameter('in_topic', '/scan').value
        out_topic = self.declare_parameter('out_topic', '/scan_relay').value

        relq = QoSProfile(depth=5)
        relq.reliability = ReliabilityPolicy.RELIABLE
        relq.history = HistoryPolicy.KEEP_LAST
        self.pub = self.create_publisher(LaserScan, out_topic, relq)
        self.create_subscription(LaserScan, in_topic, self.cb, qos_profile_sensor_data)
        self.n = 0
        self.get_logger().info(
            f'scan_relay: {in_topic} → {out_topic} (step={self.step}, '
            f'drop_intensities={self.drop_intensities}, RELIABLE)')

    def cb(self, m):
        s = self.step
        o = LaserScan()
        o.header = m.header
        o.angle_min = m.angle_min
        o.angle_max = m.angle_max
        o.angle_increment = m.angle_increment * s
        o.time_increment = m.time_increment * s
        o.scan_time = m.scan_time
        o.range_min = m.range_min
        o.range_max = m.range_max
        o.ranges = list(m.ranges[::s])
        o.intensities = [] if self.drop_intensities else list(m.intensities[::s])
        self.pub.publish(o)
        self.n += 1
        if self.n % 100 == 0:
            self.get_logger().info(f'relayed {self.n}, pts={len(o.ranges)}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
