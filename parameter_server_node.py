import rclpy
from rclpy.node import Node

class ParameterServerNode(Node):
    def __init__(self):
        super().__init__('parameter_server_node')
        self.declare_parameter('link_lengths', [100.0, 100.0])

def main(args=None):
    rclpy.init(args=args)
    node = ParameterServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
