import rclpy
from rclpy.node import Node


class IMU_INS_Node(Node):
    def __init__(self):
        super().__init__('imu_ins_node')
        self.get_logger().info('IMU_INS_Node started')


def main(args=None):
    rclpy.init(args=args)
    node = IMU_INS_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
