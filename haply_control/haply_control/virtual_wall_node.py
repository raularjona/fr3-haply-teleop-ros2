import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped


class VirtualWallNode(Node):

    def __init__(self):
        super().__init__('virtual_wall_node')

        # Subscriber: posición del Haply
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'haply_pose',
            self.pose_callback,
            10
        )

        # Publisher: fuerza al Haply
        self.force_pub = self.create_publisher(
            WrenchStamped,
            'haply_force',
            10
        )

        # Parámetros de la pared
        self.wall_x = 0.05        # posición de la pared
        self.stiffness = 400.0    # rigidez (N/m)

        self.get_logger().info("Virtual Wall Node iniciado.")

    def pose_callback(self, msg: PoseStamped):

        x = msg.pose.position.x

        penetration = x - self.wall_x

        force_x = 0.0

        if penetration > 0:
            force_x = -self.stiffness * penetration

        wrench = WrenchStamped()
        wrench.header.stamp = self.get_clock().now().to_msg()

        wrench.wrench.force.x = force_x
        wrench.wrench.force.y = 0.0
        wrench.wrench.force.z = 0.0

        self.force_pub.publish(wrench)


def main(args=None):

    rclpy.init(args=args)

    node = VirtualWallNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
