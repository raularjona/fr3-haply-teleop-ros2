import asyncio
import websockets
import orjson
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class HaplyRosNode(Node):
    def __init__(self):
        super().__init__('haply_bridge_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'haply_pose', 10)
        self.get_logger().info('Nodo Haply Bridge iniciado. Esperando datos del WebSocket...')

    def publish_data(self, pos, orient):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base' # O el frame que prefieras
        
        # Posición (mapeo directo del WebSocket)
        msg.pose.position.x = float(pos.get('x', 0.0))
        msg.pose.position.y = float(pos.get('y', 0.0))
        msg.pose.position.z = float(pos.get('z', 0.0))
        
        # Orientación (si el Verse Grip la da, si no, neutra)
        msg.pose.orientation.x = float(orient.get('x', 0.0))
        msg.pose.orientation.y = float(orient.get('y', 0.0))
        msg.pose.orientation.z = float(orient.get('z', 0.0))
        msg.pose.orientation.w = float(orient.get('w', 1.0))
        
        self.publisher_.publish(msg)

async def haply_client(ros_node):
    uri = 'ws://localhost:10001'
    inverse3_device_id = None
    force = {"x": 0, "y": 0, "z": 0}

    async with websockets.connect(uri) as ws:
        while rclpy.ok():
            response = await ws.recv()
            data = orjson.loads(response)

            inverse3_devices = data.get("inverse3", [])
            verse_grip_devices = data.get("wireless_verse_grip", [])

            inverse3_data = inverse3_devices[0] if inverse3_devices else {}
            verse_grip_data = verse_grip_devices[0] if verse_grip_devices else {}

            if not inverse3_device_id and inverse3_data:
                inverse3_device_id = inverse3_data.get("device_id")
                ros_node.get_logger().info(f"Conectado a Inverse3 ID: {inverse3_device_id}")

            # Extraer datos
            position = inverse3_data.get("state", {}).get("cursor_position", {})
            orientation = verse_grip_data.get("state", {}).get("orientation", {})

            # PUBLICAR EN ROS 2
            if position:
                ros_node.publish_data(position, orientation)

            # Enviar fuerza (obligatorio para seguir recibiendo datos)
            request_msg = {
                "inverse3": [{
                    "device_id": inverse3_device_id,
                    "commands": {"set_cursor_force": {"values": force}}
                }]
            }
            await ws.send(orjson.dumps(request_msg))
            
            # Un pequeño respiro para el procesador
            await asyncio.sleep(0.001)

def main(args=None):
    rclpy.init(args=args)
    ros_node = HaplyRosNode()

    try:
        asyncio.run(haply_client(ros_node))
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()