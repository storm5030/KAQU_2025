#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float32MultiArray

class TorquePublisher(Node):
    """
    Wrench 토픽 구독
    각 토크를 병합하여 퍼블리시
    """

    def __init__(self):
        super().__init__('torque_publisher')

        # 4개 관절의 토크값 섭스크라이브 (세팅)
        self.topics = [
            '/force_torque/fl_14',
            '/force_torque/fr_14',
            '/force_torque/rl_14',
            '/force_torque/rr_14',
        ]

        # 퍼블리시용 배열
        self.tau_x = [0.0, 0.0, 0.0, 0.0]
        self.valid = [False, False, False, False]

        # 퍼블리셔 선언
        self.publisher = self.create_publisher(Float32MultiArray, '/torque_14', 10)
        self.subs = []

        # 4개 관절로부터 섭스크라이브
        for i, t in enumerate(self.topics):
            sub = self.create_subscription(Wrench, t, lambda msg, idx=i: self.callback(msg, idx), 50)
            self.subs.append(sub)
            self.get_logger().info(f'[Sub {i}] {t} (Wrench)')
        self.get_logger().info('[Pub] /torque_14 (Float32MultiArray[4])')

    # 섭스크라이브 콜백함수, 조인트 개수만큼 호출됨
    def callback(self, msg: Wrench, idx: int):
        self.tau_x[idx] = float(msg.torque.x)
        self.valid[idx] = True
        
        # 모든 값이 들어왔을 때만 퍼블리시
        if all(self.valid):
            out = Float32MultiArray()
            out.data = list(self.tau_x)
            self.publisher.publish(out)
            for i in out.data:
                print(i)
            print()
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
