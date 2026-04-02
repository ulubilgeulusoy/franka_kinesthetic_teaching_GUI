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
JOINT_STATE_TOPICS = [
    '/NS_1/franka/joint_states',
    '/NS_1/joint_states',
    '/joint_states',
]
TOPIC_PRIORITY = {topic: index for index, topic in enumerate(JOINT_STATE_TOPICS)}

class JointRecorder(Node):
    def __init__(self):
        super().__init__('joint_recorder')
        self.subscriptions = []
        for topic in JOINT_STATE_TOPICS:
            self.subscriptions.append(
                self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, source_topic=topic: self.listener_callback(msg, source_topic),
                    10,
                )
            )
        self.joint_data = []
        self.start_time = None
        self.active_joint_topic = None

    def listener_callback(self, msg, source_topic):
        joint_map = dict(zip(msg.name, msg.position))
        missing = [joint_name for joint_name in ARM_JOINT_NAMES if joint_name not in joint_map]
        if missing:
            return

        if self.active_joint_topic is None:
            self.active_joint_topic = source_topic
            self.start_time = self.get_clock().now().nanoseconds
            self.get_logger().info(f"Recording joint states from {source_topic}")
        elif TOPIC_PRIORITY[source_topic] < TOPIC_PRIORITY[self.active_joint_topic]:
            self.active_joint_topic = source_topic
            self.start_time = self.get_clock().now().nanoseconds
            self.joint_data = []
            self.get_logger().info(f"Switching recording joint state source to preferred topic {source_topic}")
        elif source_topic != self.active_joint_topic:
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
