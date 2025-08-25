
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import json


with open("map.json", "r", encoding="utf-8") as f:
    map_data = json.load(f)

nodes = map_data["nodes"]
edges = map_data["edges"]



class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.get_logger().info('✅ TestNode has started successfully!')

def main(args=None):
    rclpy.init(args=args)
    # node = Navigator()
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
