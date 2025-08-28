import rclpy
import json
import math
import time
import networkx as nx # Dijkstra 알고리즘
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.state_publisher = self.create_publisher(String, '/cmd_state', 10)
        self.control_publisher = self.create_publisher(Twist, '/cmd_control', 10)
        self.create_subscription(Point, 'destination_coordinates', self.destination_callback, 10)

        with open('map.json', 'r', encoding='utf-8') as f:
            self.map_data = json.load(f)
        self.G = self.create_graph(self.map_data)

        self.current_node_id = 1001  # 시작 위치: 1층 현관 입구
        self.current_orientation_deg = 90.0 # 초기 방향: 북쪽

        self.LINEAR_VELOCITY = 0.2  # m/s
        self.ANGULAR_VELOCITY = 0.5  # rad/s

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
            
        goal_node_info = self.get_node_by_id(goal_node_id)
        
        try:
            path = nx.shortest_path(self.G, source=start_node_id, target=goal_node_id, weight='weight')
            #print("path:", path) 
            self.execute_path(path)
        except (nx.NetworkXNoPath, nx.NodeNotFound) as e:
            self.get_logger().error(f"경로를 찾을 수 없습니다: {e}")

    def find_closest_node(self, x, y, z):
        min_dist = float('inf')
        closest_node_id = None
        for node in self.map_data['nodes']:
            if node['z'] == int(z): # 같은 층에 있는 노드만 대상
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
        # TROT 상태로 변경
        self.set_cmd_state("TROT")
        self.get_logger().info("이동을 시작합니다")
        time.sleep(2)

        for i in range(len(path) - 1):
            from_node = self.get_node_by_id(path[i])
            to_node = self.get_node_by_id(path[i+1])

            # 다른 층 이동
            if from_node['z'] != to_node['z']:
                self.get_logger().info(f"{from_node['z']}층에서 {to_node['z']}층으로 이동합니다...")
                time.sleep(5)
            
            # 같은 층 이동
            else:
                target_angle_deg = math.degrees(math.atan2(to_node['y'] - from_node['y'], to_node['x'] - from_node['x']))
                turn_angle_deg = target_angle_deg - self.current_orientation_deg
                turn_angle_deg = (turn_angle_deg + 180) % 360 - 180
                self.robot_turn(turn_angle_deg)
                self.current_orientation_deg = target_angle_deg

                distance_mm = math.sqrt((to_node['x'] - from_node['x'])**2 + (to_node['y'] - from_node['y'])**2)
                self.robot_forward(distance_mm)

            self.current_node_id = to_node['id']
            self.get_logger().info(f"노드 {self.current_node_id} ({to_node['name']}) 도착")

        # 이동 완료 후, REST 상태로 변경
        self.get_logger().info("최종 목적지에 도착했습니다")
        self.set_cmd_state("REST")
    
    def set_cmd_state(self, mode):
        msg = String()
        msg.data = mode
        self.state_publisher.publish(msg)

    def robot_turn(self, turn_angle_deg):
        if abs(turn_angle_deg) < 1.0: # 작은 각도는 무시
            return
            
        self.get_logger().info(f"로봇 회전 시작: {turn_angle_deg:.1f}도")
        twist_msg = Twist()
        
        turn_rad = math.radians(turn_angle_deg)
        twist_msg.angular.z = self.ANGULAR_VELOCITY if turn_rad > 0 else -self.ANGULAR_VELOCITY
        duration = abs(turn_rad) / self.ANGULAR_VELOCITY
        
        self.control_publisher.publish(twist_msg)
        time.sleep(duration)
        
        twist_msg.angular.z = 0.0
        self.control_publisher.publish(twist_msg)
        self.get_logger().info("회전 완료")
        time.sleep(0.5)

    def robot_forward(self, distance_mm):
        self.get_logger().info(f"로봇 직진 시작: {distance_mm / 1000:.2f}m")
        twist_msg = Twist()
        
        distance_m = distance_mm / 1000.0
        duration = distance_m / self.LINEAR_VELOCITY
        
        twist_msg.linear.x = self.LINEAR_VELOCITY
        self.control_publisher.publish(twist_msg)
        time.sleep(duration)
        
        twist_msg.linear.x = 0.0
        self.control_publisher.publish(twist_msg)
        self.get_logger().info("직진 완료")
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()