#!/usr/bin/env python3
# ROS 2 Humble / Python3.10+
import json
import time
import math
from typing import List, Dict, Any

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.time import Time

from kaqu_controller.KaquCmdManager.KaquParams import LegParameters

from sensor_msgs.msg import Joy, Imu
from std_srvs.srv import Trigger

# 생성된 액션 인터페이스
from kaqu_msgs.action import FollowPath

from kaqu_nav.pose_estimator import ImuPose2D, PoseEstimatorConfig


class FollowPathServer(Node):
    def __init__(self):
        super().__init__('follow_path_server')

        leg_params = LegParameters()     
        trot = leg_params.gait           #KaquParams의 Trot 관련 변수들      

        # ===== 파라미터 (kaquParams 성격) =====
        # 전진 속도 [m/s] (양수)
        self.x_vel = trot.max_x_vel * 0.001 # mm/s -> m/s 변환
        # yaw 속도 [deg/s] (양수)
        self.yaw_rate_deg_s = np.degrees(trot.max_yaw_rate)

        # Joy 퍼블리시 주기 [Hz]
        ts = float(trot.time_step)  # 예: 0.02 s
        self.pub_hz = int(round(1.0 / ts)) if ts > 0.0 else 50
        
        # Joy 토픽명
        self.joy_topic = '/joy'

        # Joy 축 인덱스 (컨트롤러 매핑에 맞춰 조정)
        self.idx_lin = 4
        self.idx_yaw = 6

        # (선택) LLM 서비스 콜 사용 여부/네임
        self.use_llm_notify_service = False
        self.llm_srv_name = '/llm/notify_goal_done'

        self.joy_pub = self.create_publisher(Joy, self.joy_topic, 10)
        self.llm_cli = self.create_client(Trigger, self.llm_srv_name) if self.use_llm_notify_service else None

        # 내부 상태 (기준점은 서버 실행 시)
        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_deg = 0.0
        self.start_time: Time = None
        
        self.use_imu_pose = True  # IMU 추정 사용 스위치 (False면 기존 dead-reckoning 사용)
        cfg = PoseEstimatorConfig(
            calib_samples=200,
            accel_includes_gravity=False,  # 보통 linear_acceleration은 중력 제거돼 옴
            vel_leak_rate=0.02,
            prefer_quat_yaw=True
        )
        self.imu_est = ImuPose2D(cfg)
        self.create_subscription(Imu, '/imu', self._on_imu, 100)  # 토픽 이름은 실제에 맞게 조정

        # 액션 서버
        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        self.get_logger().info(f'FollowPathServer is up. pub_hz={self.pub_hz}Hz, x_vel={self.x_vel:.3f}m/s, yaw_rate={self.yaw_rate_deg_s:.1f}deg/s')

    # ----- 액션 콜백 -----

    def goal_cb(self, goal_request: FollowPath.Goal) -> GoalResponse:
        try:
            steps = self._parse_route_json(goal_request.route_json)
        except Exception as e:
            self.get_logger().warn(f'Invalid route_json: {e}')
            return GoalResponse.REJECT
        if not steps:
            self.get_logger().warn('Empty steps.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle) -> CancelResponse:
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        self.get_logger().info('Executing new route...')

        # 시작 시각/자세 리셋
        self.start_time = self.get_clock().now()
        self.x_m, self.y_m, self.yaw_deg = 0.0, 0.0, 0.0
        if self.use_imu_pose:
            self.imu_est.reset()
        # Goal 파싱
        try:
            steps = self._parse_route_json(goal_handle.request.route_json)
        except Exception as e:
            msg = f'route_json parse failed: {e}'
            self.get_logger().error(msg)
            goal_handle.abort()
            return FollowPath.Result(
                success=False, message=msg, total_time_s=0.0,
                final_x_m=float(self.x_m), final_y_m=float(self.y_m), final_yaw_deg=float(self.yaw_deg)
            )

        success = True
        total_time = 0.0

        for i, step in enumerate(steps):
            if goal_handle.is_cancel_requested:
                self._stop_joy()
                goal_handle.canceled()
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
                return FollowPath.Result(
                    success=False, message='Canceled by client',
                    total_time_s=float(total_time),
                    final_x_m=float(self.x_m), final_y_m=float(self.y_m), final_yaw_deg=float(self.yaw_deg)
                )

            if 'forward_m' in step:
                distance = float(step['forward_m'])
                duration_s = abs(distance) / max(self.x_vel, 1e-6)
                lin, yaw = (1.0 if distance >= 0.0 else -1.0), 0.0
                action_name = f'forward {distance:.3f} m'
            elif 'turn_deg' in step:
                angle = float(step['turn_deg'])
                duration_s = abs(angle) / max(self.yaw_rate_deg_s, 1e-6)
                lin, yaw = 0.0, (1.0 if angle >= 0.0 else -1.0)
                action_name = f'turn {angle:.1f} deg'
            else:
                self.get_logger().warn(f'Step {i} has no forward_m / turn_deg. Skipped.')
                continue

            self.get_logger().info(f'[{i}/{len(steps)-1}] {action_name} for {duration_s:.2f}s')

            # 동기 실행
            step_ok = self._run_for_duration(goal_handle, i, duration_s, lin, yaw)
            total_time += duration_s
            if not step_ok:
                success = False
                break

        self._stop_joy()

        if success and self.use_llm_notify_service and self.llm_cli is not None:
            # 선택: 비동기 서비스 콜 대신 try/except로 블로킹 call로 바꾸거나, 그냥 생략 가능
            pass

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        msg = 'Route completed.' if success else 'Route aborted.'
        self.get_logger().info(f'{msg} (elapsed {elapsed:.2f}s)')

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return FollowPath.Result(
            success=success, message=msg, total_time_s=float(total_time),
            final_x_m=float(self.x_m), final_y_m=float(self.y_m), final_yaw_deg=float(self.yaw_deg)
        )
    # ----- 유틸 -----

    # 클래스 메서드로 추가
    def _on_imu(self, msg: Imu):
        self.imu_est.update(msg)
    

    def _parse_route_json(self, route_json: str) -> List[Dict[str, Any]]:
        steps = json.loads(route_json)
        if not isinstance(steps, list):
            raise ValueError('route_json must be a list.')
        filtered = []
        for s in steps:
            if not isinstance(s, dict):
                continue
            # forward_m XOR turn_deg
            if ('forward_m' in s) ^ ('turn_deg' in s):
                filtered.append(s)
        return filtered

    def _run_for_duration(self, goal_handle, step_index, duration_s, lin_sign, yaw_sign) -> bool:
        hz = max(self.pub_hz, 1)
        dt = 1.0 / hz
        start = self.get_clock().now()
        end = start + Duration(seconds=duration_s)

        axes_len = max(self.idx_lin, self.idx_yaw) + 1
        joy = Joy()
        joy.axes = [0.0] * max(axes_len, 8)
        joy.buttons = [0] * 12

        v = float(lin_sign) * float(self.x_vel)            # m/s
        r_deg = float(yaw_sign) * float(self.yaw_rate_deg_s)  # deg/s

        while self.get_clock().now() < end:
            if goal_handle.is_cancel_requested:
                return False

            # Joy publish
            joy.axes[self.idx_lin] = float(lin_sign)
            joy.axes[self.idx_yaw] = float(yaw_sign)
            self.joy_pub.publish(joy)

            # --- 여기부터 수정 ---
            if self.use_imu_pose and self.imu_est.ready:
                # IMU 추정이 준비되면 IMU 값을 신뢰해 사용
                self.x_m, self.y_m, self.yaw_deg = self.imu_est.get_pose()
            else:
                # 기존 dead-reckoning (IMU 준비 전 또는 use_imu_pose=False 일 때만)
                yaw_rad = math.radians(self.yaw_deg)
                self.x_m += v * dt * math.cos(yaw_rad)
                self.y_m += v * dt * math.sin(yaw_rad)
                self.yaw_deg += r_deg * dt
                if self.yaw_deg > 180.0:
                    self.yaw_deg -= 360.0
                if self.yaw_deg < -180.0:
                    self.yaw_deg += 360.0
            # --- 수정 끝 ---

            # Feedback
            remaining = (end - self.get_clock().now()).nanoseconds * 1e-9
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            fb = FollowPath.Feedback()
            fb.current_index = step_index
            fb.remaining_time_s = max(0.0, float(remaining))
            fb.elapsed_time_s = float(elapsed)
            fb.x_m = float(self.x_m)
            fb.y_m = float(self.y_m)
            fb.yaw_deg = float(self.yaw_deg)
            goal_handle.publish_feedback(fb)

            time.sleep(dt)

        return True

    def _stop_joy(self):
        axes_len = max(self.idx_lin, self.idx_yaw) + 1
        joy = Joy()
        joy.axes = [0.0] * max(axes_len, 8)
        joy.buttons = [0] * 12
        self.joy_pub.publish(joy)

def main():
    rclpy.init()
    node = FollowPathServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()