import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
from datetime import datetime

class JointRecorder(Node):
    def __init__(self):
        super().__init__('joint_recorder')
        # Subscribe to the correct joint_states topic with namespace
        self.subscription = self.create_subscription(
            JointState,
            '/NS_1/joint_states',
            self.listener_callback,
            10)
        self.joint_data = []
        self.start_time = self.get_clock().now().nanoseconds

    def listener_callback(self, msg):
        timestamp = self.get_clock().now().nanoseconds - self.start_time
        self.joint_data.append([timestamp] + list(msg.position))

    def save_to_csv(self, filename):
        if not self.joint_data:
            self.get_logger().error("No joint data recorded! Not saving CSV.")
            return
        with open(filename, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_ns'] + [f'joint_{i+1}' for i in range(len(self.joint_data[0]) - 1)])
            writer.writerows(self.joint_data)
        self.get_logger().info(f'Saved trajectory to {filename}')

def main(args=None):
    rclpy.init(args=args)
    recorder = JointRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'joint_trajectory_{timestamp}.csv'
        recorder.save_to_csv(filename)

    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
