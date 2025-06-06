import rclpy, json, socket, struct
from rclpy.node import Node
from geometry_msgs.msg import Pose
from math import sin, cos

TCP_PORT = 7654  # UART-over-TCP endpoint Sensor Node will dial

class RadarBridge(Node):
    def __init__(self):
        super().__init__('radar_bridge')
        self.get_logger().info(f'Radar Bridge starting, awaiting TCP connection on port {TCP_PORT}...')
        self.sock = socket.create_server(('0.0.0.0', TCP_PORT), backlog=1)
        self.conn = None
        self.subscription = None
        self._accept_connection()

    def _accept_connection(self):
        self.get_logger().info('Waiting for a client to connect...')
        self.conn, addr = self.sock.accept()
        self.get_logger().info(f'Connected by {addr}')
        # Subscribe to pose only after a client connects
        if self.subscription is None:
            self.subscription = self.create_subscription(
                Pose,
                '/model/human_walker/pose',
                self.cb_pose, 
                10)
            self.get_logger().info('Subscribed to /model/human_walker/pose')

    def cb_pose(self, pose_msg):
        if not self.conn:
            self.get_logger().warn('No TCP client connected, cannot send pose data.')
            # Optionally, try to re-accept connection or handle error
            # For now, we'll just skip if no connection.
            # self._accept_connection() # Be careful with recursion here
            return

        # Very crude: encode actor position as a single point in a TI raw frame
        x, y = pose_msg.position.x, pose_msg.position.y
        # Polar to TI IWR6843 Cartesian mapping (metres → mm, truncated)
        # For simplicity, directly using x, y as if they are from a top-down radar view
        frame = {"points":[{"x_mm": int(x*1000), "y_mm": int(y*1000), "z_mm":0}]}
        payload = (json.dumps(frame)+"\n").encode()
        # prepend 2-byte length like the radar UART protocol
        try:
            header = struct.pack("<H", len(payload))
            self.conn.sendall(header + payload)
            # self.get_logger().info(f'Sent radar frame for x={x:.2f}, y={y:.2f}')
        except (socket.error, BrokenPipeError) as e:
            self.get_logger().error(f'TCP send error: {e}. Client likely disconnected.')
            self.conn.close()
            self.conn = None
            # Stop subscribing or try to accept a new connection
            if self.subscription:
                self.destroy_subscription(self.subscription)
                self.subscription = None
                self.get_logger().info('Unsubscribed from /model/human_walker/pose due to client disconnect.')
            self._accept_connection() # Attempt to accept a new connection
        except Exception as e:
            self.get_logger().error(f'Unexpected error in cb_pose: {e}')

    def spin(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info('Radar Bridge shutting down...')
        finally:
            if self.conn:
                self.conn.close()
            self.sock.close()
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    radar_bridge = RadarBridge()
    radar_bridge.spin()

if __name__ == '__main__':
    main()
