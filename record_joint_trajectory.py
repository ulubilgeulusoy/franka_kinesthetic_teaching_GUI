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
TOPIC_SELECTION_TIMEOUT_SEC = 1.5

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
        self.pending_samples = {topic: [] for topic in JOINT_STATE_TOPICS}
        self.selection_started_ns = self.get_clock().now().nanoseconds
        self.selection_timer = self.create_timer(0.1, self._attempt_topic_selection)

    def _attempt_topic_selection(self):
        if self.active_joint_topic is not None:
            return

        if self.pending_samples['/NS_1/franka/joint_states']:
            self._activate_joint_topic('/NS_1/franka/joint_states')
            return

        elapsed_sec = (self.get_clock().now().nanoseconds - self.selection_started_ns) / 1e9
        if elapsed_sec < TOPIC_SELECTION_TIMEOUT_SEC or not any(self.pending_samples.values()):
            return

        available_topics = [topic for topic, samples in self.pending_samples.items() if samples]
        best_topic = min(available_topics, key=lambda topic: TOPIC_PRIORITY[topic])
        self._activate_joint_topic(best_topic)

    def _activate_joint_topic(self, source_topic):
        if self.active_joint_topic is not None:
            return
        buffered_samples = self.pending_samples.get(source_topic, [])
        if not buffered_samples:
            return
        self.active_joint_topic = source_topic
        self.start_time = buffered_samples[0][0]
        self.get_logger().info(f"Recording joint states from {source_topic}")
        self.selection_timer.cancel()
        for sample_time_ns, ordered_positions in buffered_samples:
            timestamp = sample_time_ns - self.start_time
            self.joint_data.append([timestamp] + ordered_positions)
        self.pending_samples = {topic: [] for topic in JOINT_STATE_TOPICS}

    def listener_callback(self, msg, source_topic):
        joint_map = dict(zip(msg.name, msg.position))
        missing = [joint_name for joint_name in ARM_JOINT_NAMES if joint_name not in joint_map]
        if missing:
            return

        ordered_positions = [joint_map[joint_name] for joint_name in ARM_JOINT_NAMES]
        if self.active_joint_topic is None:
            sample_time_ns = self.get_clock().now().nanoseconds
            self.pending_samples[source_topic].append((sample_time_ns, ordered_positions))
            self._attempt_topic_selection()
            return

        if source_topic != self.active_joint_topic:
            return

        timestamp = self.get_clock().now().nanoseconds - self.start_time
        self.joint_data.append([timestamp] + ordered_positions)

    def save_to_csv(self, filename):
        if not self.joint_data and any(self.pending_samples.values()):
            available_topics = [topic for topic, samples in self.pending_samples.items() if samples]
            best_topic = min(available_topics, key=lambda topic: TOPIC_PRIORITY[topic])
            self._activate_joint_topic(best_topic)
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
