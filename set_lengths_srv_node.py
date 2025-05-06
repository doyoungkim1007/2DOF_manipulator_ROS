import rclpy
from rclpy.node import Node
from manipul_interfaces.srv import SetFloat64Array

class SetLengthsSrvNode(Node):
    def __init__(self):
        super().__init__('set_lengths_srv_node')
        self.srv = self.create_service(SetFloat64Array, 'set_lengths', self.set_lengths_callback)
        self.link_lengths = [100.0, 100.0]

    def set_lengths_callback(self, request, response):
        self.link_lengths = request.data
        response.success = True
        response.message = "Link lengths updated"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SetLengthsSrvNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
