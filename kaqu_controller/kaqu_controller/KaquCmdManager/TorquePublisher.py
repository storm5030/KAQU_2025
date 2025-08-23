#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float32MultiArray

class TorquePublisher(Node):
    """
    Wrench 토픽 구독 (힘/토크값 - 토크맨 사용)
    각 토크 벡터를 관절축 기준으로 계산하여 퍼블리시
    """

    def __init__(self):
        super().__init__('torque_publisher')

        self.topics = [
            '/force_torque/fl_14',
            '/force_torque/fr_14',
            '/force_torque/rl_14',
            '/force_torque/rr_14',
        ]

        self.tau_x = [0.0, 0.0, 0.0, 0.0]
        self.valid = [False, False, False, False]

        self.publisher = self.create_publisher(Float32MultiArray, '/torque_14', 10)
        self.subs = []
        for i, t in enumerate(self.topics):
            sub = self.create_subscription(Wrench, t, lambda msg, idx=i: self.cb(msg, idx), 50)
            self.subs.append(sub)
            self.get_logger().info(f'[Sub {i}] {t} (Wrench)')
        self.get_logger().info('[Pub] /torque_14 (Float32MultiArray[4])')

    def cb(self, msg: Wrench, idx: int):
        self.tau_x[idx] = float(msg.torque.x)
        self.valid[idx] = True
        
        if all(self.valid):
            out = Float32MultiArray()
            out.data = list(self.tau_x)
            self.publisher.publish(out)
            self.valid = [False, False, False, False]
            
def main(args=None):
    rclpy.init(args=args)
    torque_publisher = TorquePublisher()
    try:
        rclpy.spin(torque_publisher)
    except KeyboardInterrupt:
        torque_publisher.get_logger().info("Shutting down TorquePublisher .")
    finally:
        torque_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()