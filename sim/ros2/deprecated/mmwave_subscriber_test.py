#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import time

class MmWaveSubscriber(Node):
    def __init__(self):
        super().__init__('mmwave_subscriber_test')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/mmwave/pointcloud',
            self.pointcloud_callback,
            10  # QoS profile depth
        )
        self.message_count = 0
        self.start_time = time.time()
        self.get_logger().info('MmWave Subscriber started. Listening for pointcloud messages...')

    def pointcloud_callback(self, msg):
        self.message_count += 1
        elapsed = time.time() - self.start_time
        
        # Calculate message rate
        rate = self.message_count / elapsed if elapsed > 0 else 0
        
        self.get_logger().info(f'Received PointCloud2 with {len(msg.data)} points, frame_id: {msg.header.frame_id}')
        self.get_logger().info(f'Message #{self.message_count} at {rate:.2f} msgs/sec')
        
        # Print detailed info about the first few messages
        if self.message_count <= 3:
            self.get_logger().info(f'  Width: {msg.width}, Height: {msg.height}')
            self.get_logger().info(f'  Point step: {msg.point_step}, Row step: {msg.row_step}')
            self.get_logger().info(f'  Fields: {[field.name for field in msg.fields]}')

def main(args=None):
    rclpy.init(args=args)
    mmwave_subscriber = MmWaveSubscriber()
    
    try:
        rclpy.spin(mmwave_subscriber)
    except KeyboardInterrupt:
        mmwave_subscriber.get_logger().info('Subscriber stopped by user')
    except Exception as e:
        mmwave_subscriber.get_logger().error(f'Error in subscriber: {e}')
    finally:
        # Summary stats
        elapsed = time.time() - mmwave_subscriber.start_time
        rate = mmwave_subscriber.message_count / elapsed if elapsed > 0 else 0
        mmwave_subscriber.get_logger().info(
            f'Summary: Received {mmwave_subscriber.message_count} messages over {elapsed:.2f} seconds ({rate:.2f} msgs/sec)')
        
        mmwave_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
