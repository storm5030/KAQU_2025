#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from kaqu_msgs.action import FollowPath

class FollowPathClient(Node):
    def __init__(self):
        super().__init__('follow_path_client')
        self.cli = ActionClient(self, FollowPath, 'follow_path')

    def send(self, steps):
        goal = FollowPath.Goal()
        goal.route_json = json.dumps(steps)
        self.cli.wait_for_server()
        send_future = self.cli.send_goal_async(goal, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)
        return send_future

    def feedback_cb(self, fb):
        self.get_logger().info(
            f'feedback: idx={fb.feedback.current_index}, remaining={fb.feedback.remaining_time_s:.2f}s'
        )

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Result: success={result.success}, msg={result.message}, "
            f"total={result.total_time_s:.2f}s, "
            f"final pose=({result.final_x_m:.2f}, {result.final_y_m:.2f}, {result.final_yaw_deg:.1f} deg)"
        )


def main():
    rclpy.init()
    node = FollowPathClient()
    steps = [
        {"turn_deg": 90},
        {"forward_m": 0.5},
        {"turn_deg": 90},
        {"forward_m": 0.5},
        {"turn_deg": 180}
    ]

    # steps = [
    #     {"forward_m": 0.5},
    #     {"turn_deg": -90},
    #     {"forward_m": 0.5},
    #     {"turn_deg": 90}
    # ]

    # steps = [
    #     # 지하 계단 - 창의관 B114
    #     {"forward_m": 1.0},
    #     {"turn_deg": 90},
    #     {"forward_m": 9.0},
    #     {"turn_deg": -90},
    #     {"forward_m": 2.0},
    #     {"turn_deg": 90},
    #     {"forward_m": 2.0},
    #     {"turn_deg": 90},


    # ]
    # steps = [
    #엘베
    #     {"forward_m": 1.0},
        # {"turn_deg": -90},
        # {"forward_m": 6.0},
        # {"turn_deg": -90},
        # {"forward_m": 4.0},
        # {"turn_deg": 90},
    # ]
    # steps = [
    #B102
    #     {"forward_m": 1.0},
        # {"turn_deg": -90},
        # {"forward_m": 10.0},
        # {"turn_deg": 90},
    # ]
    
    future = node.send(steps)

    def done_cb(_):
        node.get_logger().info('Goal accepted, waiting for result...')

    future.add_done_callback(done_cb)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
