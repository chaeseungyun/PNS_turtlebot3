#!/usr/bin/env python3
"""
Camera Obstacle Node — RGB 단독 카메라로 바닥 위 장애물을 검출해 costmap에 투입.
(분실 전 구현 재작성, 2026-07-01. 근거: 사용자 구술 + 실측 프레임/기하)

원리 (라이다가 못 보는 얇은 책상다리·의자 바퀴 대응):
  1) 바닥 = 파란 카펫. HSV로 바닥/비바닥 분리 (H·S 위주 → 햇빛/그림자 명도 변화에 강인).
  2) 각 이미지 열(column)에서 바닥과 장애물의 '접지 경계' 픽셀을 찾음.
  3) 수평 카메라 평지 IPM으로 그 픽셀을 지면좌표로 투영:
        Z_forward = h * fy / (v - cy),   lateral = Z_forward * (u - cx) / fx
     (h=지면 위 카메라 높이, (cx,cy,fx,fy)=내부파라미터)
  4) base_footprint 기준 점들을 PointCloud2 로 발행 → costmap obstacle layer 의 observation source.

R3 게이트: `enabled`=False 면 빈 클라우드만 발행(=costmap에 장애물 안 넣음).
  AMCL 수렴 중에는 mission_control 이 False 로 두고, 수렴 후 set_parameters 로 True.

캘리브레이션: /camera_info 의 K 가 유효(0 아님)하면 그 값을 우선 사용, 아니면 아래 파라미터 추정값.
  현재 카메라는 미캘리브(K=0) → 640x480, HFOV~62° 가정 fx≈fy≈530 기본값. 캘리브 후 자동 반영.

의존: cv2(system), numpy. (cv_bridge 불필요 — 원본 /image_raw 를 numpy reshape 로 디코드)
  네트워크: 재부팅 후 원본 /image_raw 가 PC에 직접 수신됨(14.3Hz). compressed 는 fresh rclpy
  노드에서 discovery 매칭이 불안정해 raw 직접 구독으로 결정(docs/60-network-workaround.md).
"""
import math
import struct

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header


class CameraObstacle(Node):
    def __init__(self):
        super().__init__('camera_obstacle')

        p = self.declare_parameter
        # --- 토픽/프레임 ---
        self.in_topic = p('image_topic', '/image_raw').value
        self.frame_id = p('frame_id', 'base_footprint').value
        # --- 게이트 (R3) ---
        self.enabled = p('enabled', False).value
        # --- 카메라 기하 (실측: 지면 위 0.103m, 전방 0.076m, 좌우 -0.011m, 수평 정면) ---
        self.cam_h = p('camera_height', 0.103).value
        self.cam_fwd = p('camera_forward', 0.076).value
        self.cam_lat = p('camera_lateral', -0.011).value
        # --- 내부파라미터 추정 기본값(미캘리브 대비). /camera_info 유효시 덮어씀 ---
        self.fx = p('fx', 530.0).value
        self.fy = p('fy', 530.0).value
        self.cx = p('cx', 320.0).value
        self.cy = p('cy', 240.0).value
        self.have_caminfo = False
        # --- 바닥 HSV 모델 (재실측 2026-07-02, 로봇 바닥 배치 후) ---
        # 1차 재실측: 기존 가정값이 실제 카펫의 91%를 장애물로 오검출(S를 38 이상으로 가정했으나
        #   실카펫 S는 17~52). 2차 재실측(물체 배치 테스트 중, 다른 프레임): 같은 카펫인데도
        #   근거리/조명차 때문에 S가 144~212까지 관측됨 → 고정 S 상한이 프레임마다 깨짐.
        #   → S는 사실상 판별에 안 쓰고(하한만 최소 잡음), H(색상)만으로 바닥/비바닥을 가른다.
        #   (자동노출 탓에 S·V가 프레임마다 흔들리는데 H는 비교적 안정적이었음 — 실측 근거)
        self.h_lo = p('floor_h_lo', 100).value
        self.h_hi = p('floor_h_hi', 138).value
        self.s_lo = p('floor_s_lo', 10).value
        self.s_hi = p('floor_s_hi', 255).value
        self.v_lo = p('floor_v_lo', 40).value
        self.v_hi = p('floor_v_hi', 250).value
        # --- 검출 파라미터 ---
        self.col_step = p('col_step', 4).value          # 열 subsample
        self.min_range = p('min_range', 0.12).value
        self.max_range = p('max_range', 3.0).value
        self.horizon_margin = p('horizon_margin', 6).value   # v > cy+margin 만 지면
        self.min_floor_run = p('min_floor_run', 8).value     # 바닥으로 인정할 최소 연속 픽셀
        # 장애물 최소 수직 길이(px). 접지 경계 위로 이만큼 연속 비바닥이어야 실제 장애물로 인정.
        # 카펫 질감으로 생기는 1~2px 오검출 노이즈 제거용.
        self.min_obstacle_h = p('min_obstacle_h', 20).value
        # 카메라 최하단 스캔라인에 간헐적 인코딩/센서 아티팩트(순색 빨강)가 관측됨(2026-07-02 실측,
        # frame1/4에서 재현·frame2/3에선 미재현 → 간헐적). 최하단 N px는 무조건 제외하고 스캔.
        self.bottom_margin_px = p('bottom_margin_px', 6).value
        self.publish_debug = p('publish_debug', True).value

        self.pc_pub = self.create_publisher(PointCloud2, '/camera_obstacles', 10)
        self.dbg_pub = self.create_publisher(CompressedImage, '/camera_obstacle_debug', 1) \
            if self.publish_debug else None

        self.create_subscription(CameraInfo, '/camera_info', self.caminfo_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.in_topic, self.image_cb, qos_profile_sensor_data)

        self.add_on_set_parameters_callback(self._on_param)
        self.get_logger().info(
            f'camera_obstacle 시작 (enabled={self.enabled}, h={self.cam_h}m, '
            f'floor H[{self.h_lo},{self.h_hi}] S>[{self.s_lo}])')

    def _on_param(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for pr in params:
            if pr.name == 'enabled':
                self.enabled = bool(pr.value)
                self.get_logger().info(f'enabled → {self.enabled}')
        return SetParametersResult(successful=True)

    def caminfo_cb(self, msg):
        k = msg.k
        if k[0] > 1.0 and k[4] > 1.0:   # fx, fy 유효(0 아님)
            self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]
            if not self.have_caminfo:
                self.get_logger().info(
                    f'캘리브 반영: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}')
                self.have_caminfo = True

    def image_cb(self, msg):
        # 게이트: 비활성이면 빈 클라우드
        if not self.enabled:
            self.pc_pub.publish(self._cloud([]))
            return
        # 원본 raw Image 디코드 (cv_bridge 없이 numpy). rgb8/bgr8 지원 → 이후 파이프라인은 BGR 관례.
        if msg.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().warn(
                f'지원하지 않는 encoding: {msg.encoding} (rgb8/bgr8만 지원)', throttle_duration_sec=5.0)
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img.copy()
        H, W = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        floor = cv2.inRange(hsv,
                            (self.h_lo, self.s_lo, self.v_lo),
                            (self.h_hi, self.s_hi, self.v_hi))
        # 잡음 제거: 바닥 마스크의 작은 구멍(카펫 얼룩)을 메우고, 작은 비바닥 조각을 제거
        floor = cv2.medianBlur(floor, 5)
        floor = cv2.morphologyEx(floor, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        floor = cv2.morphologyEx(floor, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        v_start = int(self.cy + self.horizon_margin)   # 이 아래만 지면
        v_bottom = H - 1 - self.bottom_margin_px        # 최하단 아티팩트 라인 제외
        points = []
        dbg = img if self.dbg_pub is not None else None

        for u in range(0, W, self.col_step):
            col = floor[:, u]
            # 바닥에서 위로: 하단부터 연속 바닥을 지나 첫 장애물 = 접지 경계
            contact_v = None
            run = 0
            v = v_bottom
            # 1) 하단에서 연속 바닥 블록 찾기
            while v > v_start and col[v] > 0:
                run += 1; v -= 1
            if run >= self.min_floor_run and v > v_start:
                contact_v = v            # 바닥 위 첫 비바닥
            elif run < self.min_floor_run and col[v_bottom] == 0:
                # 바로 앞이 이미 장애물 (로봇 코앞)
                contact_v = v_bottom
            if contact_v is None or contact_v <= v_start:
                continue
            # 접지 경계 위로 연속 비바닥(장애물 몸통)이 충분히 긴지 확인 → 카펫 노이즈 제거
            ext = 0
            vv = contact_v
            while vv > v_start and col[vv] == 0:
                ext += 1
                vv -= 1
            if ext < self.min_obstacle_h:
                continue
            Z = self.cam_h * self.fy / (contact_v - self.cy)   # 전방 거리(m)
            if Z < self.min_range or Z > self.max_range:
                continue
            lat = Z * (u - self.cx) / self.fx                  # 광학 우측(+)
            bx = self.cam_fwd + Z
            by = self.cam_lat - lat                            # base y는 좌(+) → 우측은 -
            points.append((bx, by, 0.0))
            if dbg is not None:
                cv2.circle(dbg, (u, int(contact_v)), 2, (0, 0, 255), -1)

        self.pc_pub.publish(self._cloud(points))
        if dbg is not None:
            cv2.putText(dbg, f'obst={len(points)} enabled={self.enabled}',
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            ok, enc = cv2.imencode('.jpg', dbg)
            if ok:
                m = CompressedImage(); m.header.frame_id = self.frame_id
                m.header.stamp = self.get_clock().now().to_msg()
                m.format = 'jpeg'; m.data = enc.tobytes()
                self.dbg_pub.publish(m)

    def _cloud(self, points):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = b''.join(struct.pack('fff', *p) for p in points)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CameraObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
