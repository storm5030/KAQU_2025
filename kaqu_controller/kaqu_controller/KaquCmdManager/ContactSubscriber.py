# torque_array_subscriber.py
#!/usr/bin/env python3
from typing import Callable, List, Optional
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

class ContactSubscriber:
    """
    여러 개의 Contact 토픽을 받아 접촉 여부를 전달하는 헬퍼.
    - topics: 구독할 토픽 이름 리스트
    - on_full: 모든 토픽의 최신 샘플을 한 번씩 받은 시점에 호출되는 콜백 (List[float] 인자)
    - queue_size: ROS2 subscription 큐 크기 (기본 50)
    """
    def __init__(self,
                 node: Node,
                 topics: List[str],
                 on_full: Callable[[List[float]], None],
                 queue_size: int = 50):
        self._node = node
        self._on_full = on_full

        self._N = len(topics)
        self._tau = [0.0] * self._N
        self._valid = [False] * self._N
        self._subs = []
        self._cb_group = ReentrantCallbackGroup()

        for i, t in enumerate(topics):
            sub = node.create_subscription(
                Contact, t,
                lambda msg, idx=i: self._cb(msg, idx),
                queue_size,
                callback_group=self._cb_group
            )
            self._subs.append(sub)
            # node.get_logger().info(f'[TorqueArraySubscriber] Sub {i}: {t}')

    def _cb(self, msg: Wrench, idx: int):
        # getattr로 torque.x / torque.y / torque.z 선택
        val = getattr(msg.torque, self._axis, 0.0)
        self._tau[idx] = float(val)
        self._valid[idx] = True

        if all(self._valid):
            # 콜백에 복사본 전달 (외부에서 안전하게 사용)
            self._on_full(list(self._tau))
            # 다음 배치를 기다리도록 리셋
            self._valid = [False] * self._N

    # 선택적 편의 함수들 (원하면 사용)
    def latest_vector(self) -> List[float]:
        """가장 최근에 수신한 토크 벡터(부분 수신 포함)."""
        return list(self._tau)

    def get(self, i: int) -> float:
        """i번째 최신 토크."""
        return float(self._tau[i])
