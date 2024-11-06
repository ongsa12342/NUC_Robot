#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from datetime import datetime
import yaml

class SaveTopicData(Node):
    def __init__(self):
        super().__init__('save_topic_data_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/my_response_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Prepare the data with timestamp
        data_with_timestamp = {
            'timestamp': datetime.now().isoformat(),
            'layout': {
                'dim': [],
                'data_offset': msg.layout.data_offset
            },
            'data': msg.data
        }

        # Save to YAML file
        with open('saved_data_with_timestamps.yaml', 'a') as f:
            yaml.dump([data_with_timestamp], f, default_flow_style=False)
            f.write('---\n')  # YAML document separator

        self.get_logger().info("Data saved with timestamp.")

def main(args=None):
    rclpy.init(args=args)
    node = SaveTopicData()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
