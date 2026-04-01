import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import sys
from datetime import datetime

ARM_JOINT_NAMES = [
    'fr3_joint1',
    'fr3_joint2',
    'fr3_joint3',
    'fr3_joint4',
    'fr3_joint5',
    'fr3_joint6',
    'fr3_joint7',
]

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
        joint_map = dict(zip(msg.name, msg.position))
        missing = [joint_name for joint_name in ARM_JOINT_NAMES if joint_name not in joint_map]
        if missing:
            self.get_logger().warn(f"Skipping joint sample; missing expected arm joints: {missing}")
            return

        timestamp = self.get_clock().now().nanoseconds - self.start_time
        ordered_positions = [joint_map[joint_name] for joint_name in ARM_JOINT_NAMES]
        self.joint_data.append([timestamp] + ordered_positions)

    def save_to_csv(self, filename):
        if not self.joint_data:
            self.get_logger().error("No joint data recorded! Not saving CSV.")
            return
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp_ns',
                'row_type',
                'event',
            ] + ARM_JOINT_NAMES)
            for row in self.joint_data:
                writer.writerow([row[0], 'joint', ''] + row[1:])
        self.get_logger().info(f'Saved trajectory to {filename}')

def build_output_filename():
    if len(sys.argv) >= 2 and sys.argv[1].strip():
        filename = sys.argv[1].strip()
        if not filename.endswith('.csv'):
            filename += '.csv'
        return filename

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f'joint_trajectory_{timestamp}.csv'


def main(args=None):
    rclpy.init(args=args)
    recorder = JointRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.save_to_csv(build_output_filename())
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
