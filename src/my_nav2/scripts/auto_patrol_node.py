#!/usr/bin/env python3
"""
Auto Patrol Node
- AMCL 초기화 (Home 위치 발행)
- 파티클 수렴 확인
- Waypoint 순회 실행
"""

import math
import time
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from nav2_msgs.action import NavigateToPose
from ament_index_python.packages import get_package_share_directory


class AutoPatrol(Node):
    def __init__(self):
        super().__init__('auto_patrol')
        
        # 1. Waypoint 설정 로드
        self.load_waypoints()
        
        # 2. Publisher: 초기 위치 발행용
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        
        # 3. Subscriber: AMCL 파티클 (수렴 확인용)
        self.particle_sub = self.create_subscription(
            PoseArray, '/particle_cloud', 
            self.particle_callback, 10)
        
        # 4. Action Client: Nav2 NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # 상태 변수
        self.particles = None
        self.is_converged = False
        self.current_waypoint_idx = 0
        
        # 시작 (3초 후, Nav2 활성화 대기)
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
    
    def startup(self):
        """초기 시퀀스: 한 번만 실행"""
        self.startup_timer.cancel()  # 타이머 중지
        
        # Step 1: Home 위치 발행
        self.publish_initial_pose()
        
        # Step 2: 수렴 대기 → 그 후 순회 시작
        self.get_logger().info('AMCL 수렴 대기 중...')
        self.convergence_timer = self.create_timer(1.0, self.check_and_start)
    
    def publish_initial_pose(self):
        """Home 위치를 /initialpose로 발행"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = float(self.home['x'])
        msg.pose.pose.position.y = float(self.home['y'])
        
        # yaw → quaternion
        yaw = float(self.home['yaw'])
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        # Covariance (위치 불확실성)
        msg.pose.covariance[0] = 0.25      # x 분산
        msg.pose.covariance[7] = 0.25      # y 분산
        msg.pose.covariance[35] = 0.068    # yaw 분산
        
        self.pose_pub.publish(msg)
        self.get_logger().info(
            f'Home 위치 발행: ({self.home["x"]}, {self.home["y"]}, {self.home["yaw"]})')
    
    def particle_callback(self, msg):
        """AMCL 파티클 수신"""
        self.particles = msg.poses
    
    def check_convergence(self):
        """파티클들이 한 곳에 모였는지 확인"""
        if self.particles is None or len(self.particles) < 10:
            return False
        
        # 모든 파티클의 평균 위치
        mean_x = sum(p.position.x for p in self.particles) / len(self.particles)
        mean_y = sum(p.position.y for p in self.particles) / len(self.particles)
        
        # 분산 계산 (평균에서의 거리들)
        distances = [
            math.sqrt((p.position.x - mean_x)**2 + (p.position.y - mean_y)**2)
            for p in self.particles
        ]
        
        max_distance = max(distances)
        avg_distance = sum(distances) / len(distances)
        
        # 모든 파티클이 평균에서 0.3m 이내면 수렴
        is_converged = max_distance < 0.3
        
        self.get_logger().info(
            f'파티클 분산: avg={avg_distance:.2f}m, max={max_distance:.2f}m, '
            f'수렴={is_converged}')
        
        return is_converged
    
    def check_and_start(self):
        """수렴 확인 후 순회 시작"""
        if self.check_convergence():
            self.convergence_timer.cancel()
            self.get_logger().info('AMCL 수렴 완료. 순회 시작.')
            time.sleep(1.0)
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
        
        # Action goal 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(wp['x'])
        goal_msg.pose.pose.position.y = float(wp['y'])
        yaw = float(wp['yaw'])
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        # Action 서버 대기 후 send
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
        
        # 결과 기다리기
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Nav2가 도착했거나 실패했을 때"""
        result = future.result().result
        status = future.result().status
        
        wp = self.waypoints[self.current_waypoint_idx]
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"도착! → {wp['name']}")
        else:
            self.get_logger().warning(f"실패 (status={status}) → {wp['name']}")
        
        # 다음 waypoint로
        self.current_waypoint_idx += 1
        
        # 잠시 대기 후 다음
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
