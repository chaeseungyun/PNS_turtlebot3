#!/usr/bin/env python3
"""
Auto Patrol Node  (복구·재구성 2026-06-30, 근거: 분실 전 노트 2026-06-09/06-10)

시퀀스:
  1. Home 위치를 /initialpose 로 발행 (AMCL 초기화)
  2. behavior_server 가 'active' 될 때까지 get_state 폴링      ← [노트 06-10]
  3. Nav2 Spin 액션으로 ±π 번갈아 제자리 회전 → AMCL 수렴 유도  ← [노트 06-09]
  4. /particle_cloud 로 수렴 확인되면 Waypoint 순회 시작

복구 시 베이스라인(2026-06-08) 대비 고친 점:
  (1) /particle_cloud 구독: PoseArray + 기본 QoS  →  nav2_msgs/ParticleCloud + best_effort
      Humble AMCL 은 particle_cloud 를 nav2_msgs/ParticleCloud 타입·SensorData(best_effort)
      QoS 로 발행한다. 베이스라인은 타입·QoS 둘 다 불일치라 콜백이 한 번도 안 들어와
      수렴 감지 실패 → 순찰이 영영 시작 못 했음. ("한 번도 시작 못하던 버그")
  (2) 위치추정 자동 스핀(±π) 추가. 정지 상태에선 AMCL 이 잘 안 모이고, 한 방향 2π 는
      odom 드리프트가 누적됨 → ±π 왕복.
  (3) 스핀 goal 전송 전 behavior_server 'active' 를 get_state 로 확인.
      라이프사이클상 active 이전엔 액션 goal 이 REJECT 되고, wait_for_server() 는
      서버 '존재'만 확인하지 active 여부는 모름 → 그래서 거부가 났었음.
"""

import math
import time
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.msg import ParticleCloud           # [수정1] PoseArray 아님
from lifecycle_msgs.srv import GetState           # [수정3] behavior_server active 확인
from ament_index_python.packages import get_package_share_directory

# 수렴 판정: 모든 파티클이 평균에서 이 거리(m) 이내면 수렴으로 본다.
CONVERGE_MAX_DIST = 0.3
# 스핀 1회 회전량(rad). ±π 를 번갈아 준다.
SPIN_YAW = math.pi
# 수렴 못 하면 최대 이만큼만 스핀하고 포기(무한루프 방지).
MAX_SPINS = 8


class AutoPatrol(Node):
    def __init__(self):
        super().__init__('auto_patrol')

        # 1. Waypoint 설정 로드
        self.load_waypoints()

        # 2. Publisher: 초기 위치 발행용
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # 3. Subscriber: AMCL 파티클 (수렴 확인용)
        #    [수정1] 타입 ParticleCloud + best_effort QoS 로 맞춤
        self.particle_sub = self.create_subscription(
            ParticleCloud, '/particle_cloud',
            self.particle_callback, qos_profile_sensor_data)

        # 4. Action Client: Nav2 NavigateToPose(순회), Spin(위치추정 회전)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, '/spin')

        # 5. behavior_server 라이프사이클 상태 조회 서비스 [수정3]
        self.behavior_state_cli = self.create_client(
            GetState, '/behavior_server/get_state')

        # 상태 변수
        self.particles = None
        self.is_converged = False
        self.current_waypoint_idx = 0
        self.spin_count = 0

        self.get_logger().info('Auto Patrol Node 시작. 3초 후 초기화 시작.')
        self.startup_timer = self.create_timer(3.0, self.startup)

    def load_waypoints(self):
        """waypoints.yaml 로드"""
        config_path = os.path.join(
            get_package_share_directory('my_nav2'),
            'config', 'waypoints.yaml')

        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.home = cfg['home']
        self.waypoints = cfg['waypoints']
        self.patrol_cfg = cfg['patrol']

        self.get_logger().info(
            f'로드됨: home + {len(self.waypoints)} waypoints')

    # ---------------- 초기화 + 위치추정 ----------------

    def startup(self):
        """초기 시퀀스: 한 번만 실행"""
        self.startup_timer.cancel()
        self.publish_initial_pose()
        # behavior_server 가 active 될 때까지 기다렸다가 스핀 시작 [수정3]
        self.get_logger().info('behavior_server active 대기 중...')
        self.behavior_wait_timer = self.create_timer(1.0, self.wait_behavior_active)

    def publish_initial_pose(self):
        """Home 위치를 /initialpose로 발행"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = float(self.home['x'])
        msg.pose.pose.position.y = float(self.home['y'])

        yaw = float(self.home['yaw'])
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)

        msg.pose.covariance[0] = 0.25      # x 분산
        msg.pose.covariance[7] = 0.25      # y 분산
        msg.pose.covariance[35] = 0.068    # yaw 분산

        self.pose_pub.publish(msg)
        self.get_logger().info(
            f'Home 위치 발행: ({self.home["x"]}, {self.home["y"]}, {self.home["yaw"]})')

    def wait_behavior_active(self):
        """behavior_server 가 'active' 가 되면 스핀 시작.
        wait_for_server() 는 존재만 확인하므로, get_state 로 라이프사이클을 직접 확인한다."""
        if not self.behavior_state_cli.service_is_ready():
            # 서비스 자체가 아직 안 올라옴
            return
        req = GetState.Request()
        future = self.behavior_state_cli.call_async(req)
        future.add_done_callback(self.on_behavior_state)

    def on_behavior_state(self, future):
        try:
            state_id = future.result().current_state.id
            label = future.result().current_state.label
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warning(f'get_state 실패: {e}')
            return
        # lifecycle_msgs/State.PRIMARY_STATE_ACTIVE == 3
        if state_id == 3:
            self.behavior_wait_timer.cancel()
            self.get_logger().info("behavior_server active 확인 → 위치추정 스핀 시작")
            self.do_spin()
        else:
            self.get_logger().info(f'behavior_server 상태={label}(id={state_id}), 대기...')

    def do_spin(self):
        """±π 번갈아 제자리 회전해 AMCL 수렴 유도."""
        if self.spin_count >= MAX_SPINS:
            self.get_logger().warning(
                f'{MAX_SPINS}회 스핀했지만 미수렴. 그대로 순회를 시도한다.')
            self.start_patrol()
            return

        # 매 회 방향을 뒤집어 ±π 왕복 (odom 드리프트 누적 방지)
        target = SPIN_YAW if (self.spin_count % 2 == 0) else -SPIN_YAW
        self.spin_count += 1

        goal = Spin.Goal()
        goal.target_yaw = float(target)

        if not self.spin_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/spin 액션 서버 없음. 순회로 넘어간다.')
            self.start_patrol()
            return

        self.get_logger().info(
            f'스핀 {self.spin_count}/{MAX_SPINS}: target_yaw={target:.2f} rad')
        send_future = self.spin_client.send_goal_async(goal)
        send_future.add_done_callback(self.spin_goal_response)

    def spin_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # active 확인을 거쳤는데도 거부되면 잠시 후 재시도
            self.get_logger().warning('스핀 goal 거부됨. 1초 후 재시도.')
            self.create_timer(1.0, self._retry_spin_once)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.spin_result)

    def _retry_spin_once(self):
        # 일회성 재시도 타이머 (생성 즉시 취소 후 do_spin)
        self.spin_count = max(0, self.spin_count - 1)  # 거부분은 카운트에서 제외
        self.do_spin()

    def spin_result(self, future):
        """스핀 1회 끝날 때마다 수렴 확인, 미수렴이면 반대방향 스핀."""
        if self.check_convergence():
            self.get_logger().info('AMCL 수렴 완료. 순회 시작.')
            self.start_patrol()
        else:
            self.get_logger().info('아직 미수렴 → 반대방향 스핀.')
            self.do_spin()

    def particle_callback(self, msg):
        """AMCL 파티클 수신 (nav2_msgs/ParticleCloud)."""
        self.particles = msg.particles            # [수정1] .particles[*].pose

    def check_convergence(self):
        """파티클들이 한 곳에 모였는지 확인."""
        if self.particles is None or len(self.particles) < 10:
            self.get_logger().info('파티클 수신 부족 — 수렴 판정 보류')
            return False

        xs = [p.pose.position.x for p in self.particles]
        ys = [p.pose.position.y for p in self.particles]
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)

        distances = [
            math.sqrt((x - mean_x) ** 2 + (y - mean_y) ** 2)
            for x, y in zip(xs, ys)
        ]
        max_distance = max(distances)
        avg_distance = sum(distances) / len(distances)

        is_converged = max_distance < CONVERGE_MAX_DIST
        self.get_logger().info(
            f'파티클 분산: avg={avg_distance:.2f}m, max={max_distance:.2f}m, '
            f'수렴={is_converged}')
        return is_converged

    # ---------------- 웨이포인트 순회 (베이스라인 로직 유지) ----------------

    def start_patrol(self):
        self.current_waypoint_idx = 0
        self.send_next_goal()

    def send_next_goal(self):
        """다음 waypoint로 이동 명령"""
        if self.current_waypoint_idx >= len(self.waypoints):
            if self.patrol_cfg['loop']:
                self.current_waypoint_idx = 0
                self.get_logger().info('한 사이클 완료. 다시 시작.')
            else:
                self.get_logger().info('모든 waypoint 완료. 종료.')
                return

        wp = self.waypoints[self.current_waypoint_idx]
        self.get_logger().info(
            f"이동 시작 → {wp['name']} ({wp['x']}, {wp['y']})")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        yaw = float(wp['yaw'])
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self.nav_client.wait_for_server()
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Nav2가 goal을 수락했는지 확인"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 거부됨. 다음 waypoint로.')
            self.current_waypoint_idx += 1
            self.send_next_goal()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Nav2가 도착했거나 실패했을 때"""
        status = future.result().status
        wp = self.waypoints[self.current_waypoint_idx]

        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"도착! → {wp['name']}")
        else:
            self.get_logger().warning(f"실패 (status={status}) → {wp['name']}")

        self.current_waypoint_idx += 1
        wait_time = self.patrol_cfg['wait_at_each']
        self.get_logger().info(f'{wait_time}초 대기...')
        time.sleep(wait_time)
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = AutoPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
