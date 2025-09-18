# navigate.py — 목적지 좌표 → 경로 탐색 → 회전/직진 명령 퍼블리시

import rclpy
import json
import math
import networkx as nx
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Point

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator_node')

        # 퍼블리셔
        self.ctrl_pub  = self.create_publisher(Float64MultiArray, '/cmd_control', 10)
        self.state_pub = self.create_publisher(String, '/gait_mode', 10)

        # 목적지 좌표 구독
        self.create_subscription(Point, 'destination_coordinates', self.on_destination, 10)

        # 🔹 여기서 바로 map.json 불러오기 (파라미터 X)
        try:
            with open('map.json', 'r', encoding='utf-8') as f:
                self.map_data = json.load(f)
        except Exception as e:
            self.get_logger().error(f"[INIT] map.json 로드 실패: {e}")
            raise

        self.G = self._build_graph(self.map_data)

        # 현재 상태(데모 기본값)
        self.current_node_id = 1001        # 시작 노드
        self.current_heading_deg = 90.0    # +Y 방향을 바라본다고 가정

        self.get_logger().info(f"[INIT] Navigator ready. map.json 로드 완료")

    # ────────────────────────────────────────────────
    # 그래프 만들기
    # ────────────────────────────────────────────────
    def _build_graph(self, m):
        G = nx.Graph()
        for e in m['edges']:
            u, v = e['from'], e['to']
            w = float(e.get('distance', 0.0))
            G.add_edge(u, v, weight=w)
        return G

    # ────────────────────────────────────────────────
    # 목적지 좌표 입력 → 최단 경로 계산 → 경로 실행
    # ────────────────────────────────────────────────
    def on_destination(self, p: Point):
        self.get_logger().info(f"[DEST] 입력 좌표: x={p.x:.0f}, y={p.y:.0f}, z={p.z:.0f}")

        start = self.current_node_id
        goal  = self._closest_node(p.x, p.y, p.z)
        if goal is None:
            self.get_logger().error("유효한 목적지 노드를 찾지 못했습니다.")
            return

        try:
            path = nx.shortest_path(self.G, source=start, target=goal, weight='weight')
        except (nx.NetworkXNoPath, nx.NodeNotFound) as e:
            self.get_logger().error(f"경로 계산 실패: {e}")
            return

        self.get_logger().info(f"[PATH] {path}")
        self._set_gait("TROT")
        self._execute_path(path)
        self._set_gait("REST")
        self.get_logger().info("[DONE] 목적지 도착")

    # ────────────────────────────────────────────────
    # 경로 실행: 회전각/직진거리 퍼블리시
    # ────────────────────────────────────────────────
    def _execute_path(self, path):
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            from_node = self._node_by_id(u)
            to_node   = self._node_by_id(v)

            if from_node['z'] != to_node['z']:
                self.get_logger().info(f"[FLOOR] {from_node['z']}층 → {to_node['z']}층 (계단/엘베 구간)")
                self.current_node_id = v
                continue

            target_deg = math.degrees(math.atan2(
                to_node['y'] - from_node['y'],
                to_node['x'] - from_node['x']
            ))
            turn_deg = (target_deg - self.current_heading_deg + 180) % 360 - 180
            self.current_heading_deg = target_deg

            dist_mm = float(self.G.edges[u, v]['weight'])  # 🔹 지도 distance 그대로 사용

            msg = Float64MultiArray()
            msg.data = [turn_deg, dist_mm]
            self.ctrl_pub.publish(msg)

            self.current_node_id = v
            self.get_logger().info(f"[STEP] {u}→{v} | turn={turn_deg:.1f}°, fwd={dist_mm:.0f}mm")

    # ────────────────────────────────────────────────
    # 유틸 함수
    # ────────────────────────────────────────────────
    def _closest_node(self, x, y, z):
        best, best_id = float('inf'), None
        for n in self.map_data['nodes']:
            if int(n['z']) != int(z):
                continue
            d = (n['x'] - x) ** 2 + (n['y'] - y) ** 2
            if d < best:
                best, best_id = d, n['id']
        return best_id

    def _node_by_id(self, nid):
        for n in self.map_data['nodes']:
            if n['id'] == nid:
                return n
        raise KeyError(f"node id {nid} not in map")

    def _set_gait(self, mode: str):
        msg = String()
        msg.data = mode
        self.state_pub.publish(msg)
        self.get_logger().info(f"[GAIT] {mode}")

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
