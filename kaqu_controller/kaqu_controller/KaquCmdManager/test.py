
import rclpy
from rclpy.node import Node
from kaqu_controller.KaquCmdManager.RobotManagerNode import RobotManager
from kaqu_controller.KaquCmdManager.KaquParams import RobotState, RobotCommand


class DebugRobotStateCommandNode(Node):
    def __init__(self):
        super().__init__('debug_robot_state_command_node')

        body = [100., 100.]  # Body dimensions
        legs = [100., 100., 100., 100.]  # Leg dimensions
        USE_IMU = True

        # RobotManager 객체 초기화
        self.robot_manager = RobotManager(body, legs, USE_IMU)

        # Timer로 주기적으로 디버깅 함수 호출
        self.timer = self.create_timer(0.1, self.log_robot_state_command)  # 0.1초 간격

    def log_robot_state_command(self):
        """RobotState와 RobotCommand 내부 값을 로그로 출력."""
        state = self.robot_manager.state  # RobotState 객체
        command = self.robot_manager.command  # RobotCommand 객체

        self.get_logger().info("==== RobotState ====")
        for attr, value in vars(state).items():
            self.get_logger().info(f"{attr}: {value}")

        self.get_logger().info("==== RobotCommand ====")
        for attr, value in vars(command).items():
            self.get_logger().info(f"{attr}: {value}")


def main(args=None):
    rclpy.init(args=args)
    node = DebugRobotStateCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
