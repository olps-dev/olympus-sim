from gz.transport14 import Node
from gz.msgs11 import pointcloud_packed_pb2
import time

# Define a simple callback function
def callback(msg):
    print(f"Received message: {msg}")

print("Starting minimal Gazebo transport test...")

# Create a Gazebo transport node
gz_node = Node()

# Define the topic and message type
topic = "/test_mmwave_points"
msg_type_class = pointcloud_packed_pb2.PointCloudPacked

print(f"Attempting to subscribe to topic: {topic}")

# Try subscribing using the message type's full name string
if hasattr(msg_type_class, 'DESCRIPTOR') and hasattr(msg_type_class.DESCRIPTOR, 'full_name'):
    msg_type_name = msg_type_class.DESCRIPTOR.full_name
    print(f"Using message type name for subscription: {msg_type_name}")
    subscribed = gz_node.subscribe(topic, callback, msg_type_name)
    if subscribed:
        print(f"Successfully subscribed to topic {topic} using message type name '{msg_type_name}'.")
    else:
        print(f"Failed to subscribe to topic {topic} using message type name '{msg_type_name}'.")
else:
    print("Error: Message type class does not have DESCRIPTOR.full_name attribute.")

print("\nTest complete. If subscribed, the script would wait for messages (but none are expected to be published here).")
print("Check for 'str object has no attribute DESCRIPTOR' or Protobuf descriptor pool errors above.")

# Keep alive for a moment if we want to see if any async errors pop up, though not strictly necessary for this test's purpose.
# time.sleep(2)
