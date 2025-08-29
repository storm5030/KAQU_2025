#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from ros_gz_interfaces.msg import Contacts

class ContactSubscriber:
    """
    여러 Contacts 토픽을 구독해 '접촉 여부(True/False)' 배열을 주기적으로 콜백으로 넘김.
    - 복잡한 채터링 억제/필터링 없음.
    - 희소 발행 대비 타임아웃만 적용.
    """
    def __init__(self, node: Node, topics, on_full,
                 sensor_hz: float = 200.0,
                 timeout_factor: float = 2.5,
                 emit_hz: float = 200.0,
                 depth: int = 50):
        self._node = node
        self._on_full = on_full
        self._N = len(topics)

        self._last_flag = [False] * self._N
        self._last_rx   = [node.get_clock().now()] * self._N

        self._cb_group = ReentrantCallbackGroup()
        base = qos_profile_sensor_data
        self._qos = QoSProfile(
            reliability=base.reliability,
            durability=base.durability,
            history=base.history,
            depth=depth
        )

        self._period_ns  = int(1e9 / sensor_hz)
        self._timeout_ns = int(self._period_ns * timeout_factor)
        self._emit_sec   = 1.0 / emit_hz

        for i, t in enumerate(topics):
            node.create_subscription(
                Contacts, t,
                lambda msg, idx=i: self._cb(msg, idx),
                self._qos,
                callback_group=self._cb_group
            )

        self._timer = node.create_timer(
            self._emit_sec, self._emit,
            callback_group=self._cb_group
        )

    def _cb(self, msg: Contacts, idx: int):
        self._last_flag[idx] = (len(msg.contacts) > 0)
        self._last_rx[idx]   = self._node.get_clock().now()

    def _emit(self):
        now = self._node.get_clock().now()
        out = []
        for i in range(self._N):
            elapsed = (now - self._last_rx[i]).nanoseconds
            if elapsed > self._timeout_ns:
                out.append(False)
            else:
                out.append(self._last_flag[i])
        self._on_full(out)
