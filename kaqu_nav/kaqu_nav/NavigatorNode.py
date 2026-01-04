#!/usr/bin/env python3
import os
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

        # ✅ 메시지 유실 방지용 QoS (구독자가 나중에 떠도 마지막 1개는 받을 수 있게)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # LLMNode가 보내는 plan(JSON string)
        self.create_subscription(String, 'destination_plan', self.plan_callback, qos)

        # ✅ 생성된 steps(JSON array string)를 퍼블리시
        self.steps_pub = self.create_publisher(String, 'generated_steps', qos)

        share_dir = get_package_share_directory('kaqu_nav')
        map_path = os.path.join(share_dir, 'map.json')
        with open(map_path, 'r', encoding='utf-8') as f:
            self.map_data = json.load(f)

        self.home_id = int(self.map_data.get("home_id", 1001))
        self.place_by_id = {int(p["id"]): p for p in self.map_data.get("places", [])}

        self.get_logger().info(f"Loaded places={len(self.place_by_id)}, home_id={self.home_id}")

        # (옵션) 파일로도 저장
        self.save_to_file = True
        self.output_dir = os.path.expanduser("~/kaqu_ws")
        os.makedirs(self.output_dir, exist_ok=True)

    def plan_callback(self, msg: String):
        try:
            plan = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"destination_plan JSON 파싱 실패: {e} / data={msg.data}")
            return

        dest_ids = plan.get("dest_ids", [])
        return_to_home = bool(plan.get("return_to_home", False))

        if not dest_ids:
            self.get_logger().error("plan.dest_ids가 비어있습니다.")
            return

        for pid in dest_ids:
            if int(pid) not in self.place_by_id:
                self.get_logger().error(f"Unknown place id: {pid}")
                return

        merged_steps = self.build_steps_home_based([int(x) for x in dest_ids], return_to_home)
        if merged_steps is None:
            return

        # ✅ 원하는 포맷: JSON 배열 string
        steps_json_str = json.dumps(merged_steps, ensure_ascii=False, indent=2)

        out = String()
        out.data = steps_json_str
        self.steps_pub.publish(out)
        self.get_logger().info(f"Published /generated_steps (len={len(merged_steps)})")

        if self.save_to_file:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_path = os.path.join(self.output_dir, f"generated_steps_{ts}.json")
            try:
                with open(out_path, "w", encoding="utf-8") as f:
                    f.write(steps_json_str)
                self.get_logger().info(f"Saved steps file: {out_path}")
            except Exception as e:
                self.get_logger().error(f"파일 저장 실패: {e}")

    def build_steps_home_based(self, dest_ids, return_to_home: bool):
        """
        루트 개념 없이 HOME 기준:
        - 목적지 여러 개면: HOME->A(go) -> HOME(back) -> HOME->B(go) -> HOME(back) ...
        - 마지막에 return_to_home=true면 마지막 목적지도 back_steps 붙임
        """
        merged = []

        for i, pid in enumerate(dest_ids):
            place = self.place_by_id[pid]
            name = place.get("name", str(pid))

            go_steps = place.get("go_steps", [])
            back_steps = place.get("back_steps", [])

            if not go_steps:
                self.get_logger().error(f"{pid} ({name}) go_steps가 없습니다.")
                return None

            merged.extend(go_steps)

            if i < len(dest_ids) - 1:
                if not back_steps:
                    self.get_logger().error(
                        f"{pid} ({name}) back_steps가 없어서 다음 목적지로 진행 불가(루트 없음)."
                    )
                    return None
                merged.extend(back_steps)

        if return_to_home:
            last_id = dest_ids[-1]
            last_place = self.place_by_id[last_id]
            last_name = last_place.get("name", str(last_id))
            back_steps = last_place.get("back_steps", [])

            if not back_steps:
                self.get_logger().error(f"return_to_home=true인데 {last_id} ({last_name}) back_steps가 없습니다.")
                return None
            merged.extend(back_steps)

        return merged


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()