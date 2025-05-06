import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointAnglesPubNode(Node):
    def __init__(self):
        super().__init__('joint_angles_pub_node')
        self.joint_pub = self.create_publisher(Float64MultiArray, 'joint_angles', 10)
        self.joint_angles = [0.0, 0.0]
        self.timer = self.create_timer(0.1, self.publish_joint_angles)

    def publish_joint_angles(self):
        msg = Float64MultiArray()
        msg.data = self.joint_angles
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointAnglesPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
