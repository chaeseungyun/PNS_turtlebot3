#!/usr/bin/env python3
"""Republish /scan as /scan_filtered with the robot self-occlusion arc masked out.

History:
  - 2026-06-05: arm mounted -> persistent short returns (~0.33-0.37m) over the
    front arc ~346 -> 0 -> 117deg. We blanked that front wedge.
  - 2026-06-08 (bringup + live verify): arm REMOVED. Real residual occlusion is
    a sharp 100%-no-return (inf) wedge at **66-107deg = robot LEFT** (scan 0deg =
    robot front, confirmed base_link->base_scan yaw=0). It is robot-fixed (stayed
    put under a 21deg in-place rotation) -> the remaining base Dynamixel at lidar
    height, sitting <range_min(0.1m) from the lidar so its echoes are dropped to
    inf. The front arc is now CLEAN (0.7-4.6m real returns) and must NOT be masked.

So this masks a single BAND [MASK_LO, MASK_HI] (default 64-109deg, the left wedge
+ a couple deg of margin), NOT a front wedge. Masked beams -> +inf ("no return")
so AMCL/costmap stop matching against the Dynamixel. ~315deg of clean view kept.

Disable masking: set mask_lo > mask_hi (e.g. lo=361, hi=0) -> pure passthrough.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

# masked if  MASK_LO <= beam_angle(deg, 0..360) <= MASK_HI   (band-pass)
MASK_LO = 64.0     # left-occlusion band lower edge (deg)
MASK_HI = 109.0    # left-occlusion band upper edge (deg)


class ArmMask(Node):
    def __init__(self):
        super().__init__('scan_arm_mask')
        self.declare_parameter('mask_lo', MASK_LO)
        self.declare_parameter('mask_hi', MASK_HI)
        self.lo = self.get_parameter('mask_lo').value
        self.hi = self.get_parameter('mask_hi').value
        self.pub = self.create_publisher(LaserScan, '/scan_filtered',
                                         qos_profile_sensor_data)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb,
                                            qos_profile_sensor_data)
        self.n_masked = None
        if self.lo > self.hi:
            self.get_logger().info(
                f'mask_lo({self.lo}) > mask_hi({self.hi}): passthrough '
                f'(/scan -> /scan_filtered unchanged)')
        else:
            self.get_logger().info(
                f'masking beams with {self.lo} <= angle <= {self.hi} deg '
                f'(robot-left self-occlusion) -> /scan_filtered')

    def cb(self, msg):
        ranges = list(msg.ranges)
        inf = float('inf')
        masked = 0
        for i in range(len(ranges)):
            deg = math.degrees(msg.angle_min + i * msg.angle_increment) % 360.0
            if self.lo <= deg <= self.hi:
                ranges[i] = inf
                masked += 1
        msg.ranges = ranges
        self.pub.publish(msg)
        if self.n_masked != masked:
            self.n_masked = masked
            self.get_logger().info(
                f'masked {masked}/{len(ranges)} beams ('
                f'{100.0*masked/len(ranges):.0f}%), forwarding rest')


def main():
    rclpy.init()
    node = ArmMask()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
