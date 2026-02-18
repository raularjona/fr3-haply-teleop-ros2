import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped # Cambiado de Pose a PoseStamped
import math

class HaplySimulator(Node):
    def __init__(self):
        super().__init__('haply_simulator')
        # Cambiamos el tipo de mensaje aquí también
        self.publisher_ = self.create_publisher(PoseStamped, 'haply_pose', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('Simulador Haply activo: Publicando PoseStamped en /haply_pose')
        self.t = 0.0

    def timer_callback(self):
        msg = PoseStamped()
        
        # Rellenamos la cabecera (Header) - ¡ESTO ES LO QUE FALTA!
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Trayectoria circular
        radius = 0.1
        msg.pose.position.x = 0.5 + radius * math.cos(self.t)
        msg.pose.position.y = 0.0 + radius * math.sin(self.t)
        msg.pose.position.z = 0.5
        
        msg.pose.orientation.x = 1.0
        msg.pose.orientation.w = 0.0

        self.publisher_.publish(msg)
        self.t += 0.02

def main(args=None):
    rclpy.init(args=args)
    node = HaplySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
