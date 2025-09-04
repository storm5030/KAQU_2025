#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Joy

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_player')
        # TODO: 필요한 파라미터 선언 (예: rate_hz, route 등)
        self.declare_parameter('rate_hz', 50.0)

        # 퍼블리셔: 실제 토픽 이름 맞게 수정
        self.pub = self.create_publisher(Joy, '/메시지 이름', 10)

        # 주기 타이머 (기본 50Hz)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info('PathPublisher started.')

    def on_timer(self):
        # TODO: 여기서 현재 단계/세그먼트에 맞는 명령 생성
        msg = Joy()
        # msg의 필드 채우기 
        # 예시) msg.vx, msg.yaw_rate 등이 있다면 여기에 설정
        # msg.vx = 0.0
        # msg.vy = 0.0
        # msg.yaw_rate = 0.0

        # 선택: 헤더가 있다면 타임스탬프 찍기
        if hasattr(msg, 'header') and isinstance(msg.header, Header):
            msg.header.stamp = self.get_clock().now().to_msg()

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
