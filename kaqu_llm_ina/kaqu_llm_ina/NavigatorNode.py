import rclpy
import json
import math
import time
import os
import networkx as nx
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Point, Twist
from ament_index_python.packages import get_package_share_directory

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.control_publisher = self.create_publisher(Float64MultiArray, '/cmd_control', 10)
        self.state_publisher = self.create_publisher(String, '/gait_mode', 10)
        
        self.create_subscription(Point, 'destination_coordinates', self.destination_callback, 10)

        share_dir = get_package_share_directory('kaqu_llm_ina')
        map_path = os.path.join(share_dir, 'map.json')
        with open(map_path, 'r', encoding='utf-8') as f:
            self.map_data = json.load(f)
        self.G = self.create_graph(self.map_data)

        self.current_node_id = 1001
        self.current_orientation_deg = 90.0

    def create_graph(self, map_data):
        G = nx.Graph()
        for edge in map_data['edges']:
            G.add_edge(edge['from'], edge['to'], weight=edge.get('distance', 0))
        return G

    def destination_callback(self, msg):
        self.get_logger().info(f"목적지 좌표: X={msg.x:.0f}, Y={msg.y:.0f}, Z={msg.z:.0f}")
        start_node_id = self.current_node_id
        goal_node_id = self.find_closest_node(msg.x, msg.y, msg.z)
        if goal_node_id is None:
            self.get_logger().error("지도에서 유효한 목적지 노드를 찾지 못했습니다.")
            return
        try:
            path = nx.shortest_path(self.G, source=start_node_id, target=goal_node_id, weight='weight')
            self.execute_path(path)
        except (nx.NetworkXNoPath, nx.NodeNotFound) as e:
            self.get_logger().error(f"경로를 찾을 수 없습니다: {e}")

    def find_closest_node(self, x, y, z):
        min_dist = float('inf')
        closest_node_id = None
        for node in self.map_data['nodes']:
            if node['z'] == int(z):
                dist = math.sqrt((node['x'] - x)**2 + (node['y'] - y)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_node_id = node['id']
        return closest_node_id

    def get_node_by_id(self, node_id):
        for node in self.map_data['nodes']:
            if node['id'] == node_id:
                return node
        return None

    def execute_path(self, path):
        self.set_cmd_state("TROT")
        self.get_logger().info("이동을 시작합니다")
        time.sleep(2)
        for i in range(len(path) - 1):
            from_node = self.get_node_by_id(path[i])
            to_node = self.get_node_by_id(path[i+1])
            if from_node['z'] != to_node['z']:
                self.get_logger().info(f"{from_node['z']}층에서 {to_node['z']}층으로 이동합니다...")
                time.sleep(5)
            else:
                target_angle_deg = math.degrees(math.atan2(to_node['y'] - from_node['y'], to_node['x'] - from_node['x']))
                turn_angle_deg = target_angle_deg - self.current_orientation_deg
                turn_angle_deg = (turn_angle_deg + 180) % 360 - 180
                self.current_orientation_deg = target_angle_deg
                distance_mm = math.sqrt((to_node['x'] - from_node['x'])**2 + (to_node['y'] - from_node['y'])**2)
                
                control_msg = Float64MultiArray()
                control_msg.data = [turn_angle_deg, distance_mm]
                self.control_publisher.publish(control_msg)
                
                time.sleep(5)

            self.current_node_id = to_node['id']
            self.get_logger().info(f"노드 {self.current_node_id} ({to_node['name']}) 도착")
        self.get_logger().info("최종 목적지에 도착했습니다")
        self.set_cmd_state("REST")
    
    def set_cmd_state(self, mode):
        msg = String()
        msg.data = mode
        self.state_publisher.publish(msg)
        self.get_logger().info(f"보행 모드를 '{mode}'(으)로 변경")

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

if __name__ == '__main__':
    main()