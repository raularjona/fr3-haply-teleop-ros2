import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped

class HaplyTranslator(Node):
    def __init__(self):
        super().__init__('haply_translator')
        
        # Suscribirse al tópico que publica tu primer nodo (haply_control)
        self.subscription = self.create_subscription(
            PoseStamped, 
            'haply_pose', 
            self.listener_callback, 
            10)
        
        # Publicar velocidades para MoveIt Servo
        self.publisher_ = self.create_publisher(
            TwistStamped, 
            '/delta_twist_cmds', 
            10)
        
        self.last_pose = None
        self.scale = 100.0  # Sensibilidad: aumenta este valor para más velocidad
        self.get_logger().info('Nodo Traductor Diferencial listo.')

    def listener_callback(self, msg):
        if self.last_pose is None:
            self.last_pose = msg
            return

        # Cálculo diferencial (Delta)
        dx = msg.pose.position.x - self.last_pose.pose.position.x
        dy = msg.pose.position.y - self.last_pose.pose.position.y
        dz = msg.pose.position.z - self.last_pose.pose.position.z

        # Crear mensaje de velocidad
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "fr3_link0" # Frame base del robot

        # Aplicar escala para convertir desplazamiento en velocidad
        twist_msg.twist.linear.x = dx * self.scale
        twist_msg.twist.linear.y = dy * self.scale
        twist_msg.twist.linear.z = dz * self.scale

        self.publisher_.publish(twist_msg)
        
        # Guardar la pose actual para la siguiente iteración
        self.last_pose = msg

def main(args=None):
    rclpy.init(args=args)
    node = HaplyTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
