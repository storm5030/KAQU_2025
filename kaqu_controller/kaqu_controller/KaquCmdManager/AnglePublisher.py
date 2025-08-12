### 최하준 버전
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from kaqu_controller.KaquIK.InverseKinematics import InverseKinematics
from kaqu_controller.KaquCmdManager.RobotManagerNode import RobotManager
from kaqu_controller.KaquCmdManager.KaquParams import RobotState
from std_msgs.msg import Float64MultiArray


class QuadrupedControllerNode(Node):
    def __init__(self):
        super().__init__('quadruped_controller_node')

        # 로봇 파라미터
        body = [210.0, 140.5] # 길이와 너비
        legs = [0.0, 42.4, 101.0, 108.9] # 다리 링크 길이가 아니라 계산에 필요한 길이임
        default_height = 160
        x_shift_front = 0 #42
        x_shift_back = 0 #-15

        USE_IMU = True

        self.inverse_kinematics = InverseKinematics(body, legs,x_shift_front,x_shift_back)
        self.state = RobotState(0.5)

        # 발끝 위치 수신
        self.foot_subscriber = self.create_subscription(Float64MultiArray, '/legpo', self.sub_call, 10)

        # Float64MultiArray 퍼블리셔 (Gazebo 및 hardware)
        self.joint_publisher = self.create_publisher(
            Float64MultiArray,
            "joint_group_position_controller/commands",
            10
        )

        # RViz용 JointState 퍼블리셔
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.joint_names = [
            "fr_mainbodyhip_joint", "fr_hip1_joint", "fr_14_joint",
            "fl_mainbodyhip_joint", "fl_hip1_joint", "fl_14_joint",
            "rr_mainbodyhip_joint", "rr_hip1_joint", "rr_14_joint",
            "rl_mainbodyhip_joint", "rl_hip1_joint", "rl_14_joint",
        ]

        # 주기적으로 main_control 호출
        self.timer = self.create_timer(0.02, self.main_control)  # 0.1초 간격
        self.leg_pos = None

    def sub_call(self, msg):
        self.leg_pos = msg.data

    def main_control(self):
        if self.leg_pos is None or len(self.leg_pos) != 12:
            self.get_logger().warn("leg_pos is not ready or invalid. Skipping control.")
            return

        try:
            leg_position = np.array(self.leg_pos).reshape((3, 4))
            self.get_logger().debug(f"Leg position reshaped: {leg_position}")
        except ValueError as e:
            self.get_logger().error(f"Error reshaping leg_pos: {e}")
            return

        dx, dy, dz = self.state.body_local_position
        roll, pitch, yaw = self.state.body_local_orientation

        try:
            # IK 계산
            pub_angles = self.inverse_kinematics.inverse_kinematics(
                leg_position, dx, dy, dz, roll, pitch, yaw
            )
            self.get_logger().info(f"Calculated joint angles: {pub_angles}")
            # numpy array를 list로 변환

            # 각도 퍼블리시
            joint_state_msg = Float64MultiArray()
            joint_state_msg.data = list(pub_angles)  # numpy array를 list로 변환
            self.joint_publisher.publish(joint_state_msg)

            # RViz용 JointState 퍼블리시
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = list(pub_angles)
            self.joint_state_pub.publish(joint_state_msg)

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()