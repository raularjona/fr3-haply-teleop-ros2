import asyncio
import websockets
import orjson
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
import time


class HaplyRosNode(Node):

    def __init__(self):
        super().__init__('haply_bridge_node')

        # Publisher pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            'haply_pose',
            10
        )

        # Subscriber fuerza
        self.force_sub = self.create_subscription(
            WrenchStamped,
            'haply_force',
            self.force_callback,
            10
        )

        self.current_force = {"x": 0.0, "y": 0.0, "z": 0.0}

        self.get_logger().info("Haply Bridge iniciado.")

        # Variables para medir frecuencia
        self.counter = 0
        self.last_time = time.time()

    def force_callback(self, msg: WrenchStamped):
        self.current_force["x"] = float(msg.wrench.force.x)
        self.current_force["y"] = float(msg.wrench.force.y)
        self.current_force["z"] = float(msg.wrench.force.z)
        self.get_logger().info(
            f"Force recibida: x={self.current_force['x']:.2f}, y={self.current_force['y']:.2f}, z={self.current_force['z']:.2f}"
        )

    def publish_pose(self, pos, orient):

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "haply_base"

        msg.pose.position.x = float(pos.get('x', 0.0))
        msg.pose.position.y = float(pos.get('y', 0.0))
        msg.pose.position.z = float(pos.get('z', 0.0))

        msg.pose.orientation.x = float(orient.get('x', 0.0))
        msg.pose.orientation.y = float(orient.get('y', 0.0))
        msg.pose.orientation.z = float(orient.get('z', 0.0))
        msg.pose.orientation.w = float(orient.get('w', 1.0))

        self.pose_pub.publish(msg)


async def haply_client(ros_node: HaplyRosNode):

    uri = "ws://localhost:10001"
    inverse3_device_id = None

    async with websockets.connect(uri, max_size=None) as ws:

        ros_node.get_logger().info("Conectado al Haply Inverse Service.")

        while rclpy.ok():
            
            # procesar callbacks ROS
            rclpy.spin_once(ros_node, timeout_sec=0)
            
            response = await ws.recv()
            data = orjson.loads(response)

            inverse3_devices = data.get("inverse3", [])
            verse_grip_devices = data.get("wireless_verse_grip", [])

            inverse3_data = inverse3_devices[0] if inverse3_devices else {}
            verse_grip_data = verse_grip_devices[0] if verse_grip_devices else {}

            if not inverse3_device_id and inverse3_data:
                inverse3_device_id = inverse3_data.get("device_id")
                ros_node.get_logger().info(
                    f"Conectado a Inverse3 ID: {inverse3_device_id}"
                )

            position = inverse3_data.get("state", {}).get("cursor_position", {})
            orientation = verse_grip_data.get("state", {}).get("orientation", {})

            if position:
                ros_node.publish_pose(position, orientation)

            # Enviar fuerza actual
            request_msg = {
                "inverse3": [{
                    "device_id": inverse3_device_id,
                    "commands": {
                        "set_cursor_force": {
                            "values": ros_node.current_force
                        }
                    }
                }]
            }

            await ws.send(orjson.dumps(request_msg))

            # Medir frecuencia real
            ros_node.counter += 1
            now = time.time()
            if now - ros_node.last_time >= 1.0:
                ros_node.get_logger().info(
                    f"Frecuencia loop Haply: {ros_node.counter} Hz"
                )
                ros_node.counter = 0
                ros_node.last_time = now


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