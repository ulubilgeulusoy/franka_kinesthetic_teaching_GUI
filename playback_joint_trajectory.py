import csv
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from franka_msgs.action import Grasp, Move
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    "fr3_joint1", "fr3_joint2", "fr3_joint3",
    "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
]
TOLERANCE = 0.05  # rad
WAIT_FOR_STATE_TIMEOUT_SEC = 15
WAIT_FOR_CONTROLLER_TIMEOUT_SEC = 20
PLAYBACK_COMPLETION_BUFFER_SEC = 2.0
SMOOTHING_WINDOW = 5  # odd number of samples for moving-average smoothing
MIN_POINT_DT = 0.03  # s, enforce minimum spacing to avoid very abrupt setpoint jumps
MIN_BLEND_TIME_SEC = 0.75
MAX_BLEND_TIME_SEC = 6.0
BLEND_SPEED_RAD_PER_SEC = 0.35
MIN_BLEND_STEPS = 10
MIN_SEGMENT_DT = 1e-3
GRIPPER_OPEN_WIDTH = 0.08
GRIPPER_CLOSE_WIDTH = 0.0
GRIPPER_SPEED = 0.1
GRIPPER_GRASP_FORCE = 40.0
GRIPPER_EPSILON_INNER = 0.005
GRIPPER_EPSILON_OUTER = 0.08
ROS_SETUP = "source /opt/ros/humble/setup.bash; source ~/franka_ws/install/setup.bash"
ROBOT_IP = "172.16.0.2"
TEACH_NAMESPACE = "NS_1"
NAMESPACE_PREFIX = f"/{TEACH_NAMESPACE}" if TEACH_NAMESPACE else ""
JOINT_STATES_TOPICS = ["/joint_states"]
TRAJECTORY_TOPICS = ["/fr3_arm_controller/joint_trajectory"]
if NAMESPACE_PREFIX:
    JOINT_STATES_TOPICS.insert(0, f"{NAMESPACE_PREFIX}/joint_states")
    TRAJECTORY_TOPICS.insert(0, f"{NAMESPACE_PREFIX}/fr3_arm_controller/joint_trajectory")
GRIPPER_MOVE_ACTION_CANDIDATES = [
    "/franka_gripper/move",
    f"/{TEACH_NAMESPACE}/franka_gripper/move",
    "/fr3_gripper/move",
    f"/{TEACH_NAMESPACE}/fr3_gripper/move",
]
GRIPPER_GRASP_ACTION_CANDIDATES = [
    "/franka_gripper/grasp",
    f"/{TEACH_NAMESPACE}/franka_gripper/grasp",
    "/fr3_gripper/grasp",
    f"/{TEACH_NAMESPACE}/fr3_gripper/grasp",
]
GRIPPER_ACTION_WAIT_TIMEOUT_SEC = 8.0


class SmartTrajectoryPlayer(Node):
    def __init__(self, csv_file):
        super().__init__("smart_trajectory_player")
        self.csv_file = csv_file
        self.trajectory_publishers = [
            self.create_publisher(JointTrajectory, topic, 10)
            for topic in TRAJECTORY_TOPICS
        ]
        self.joint_state_subscriptions = [
            self.create_subscription(JointState, topic, self.joint_state_callback, 10)
            for topic in JOINT_STATES_TOPICS
        ]

        self.actual_positions = None
        self.full_sent = False
        self.start_time_ns = self.get_clock().now().nanoseconds
        self.execution_start_time = None
        self.execution_complete_timer = None
        self.stop_requested = False
        self.gripper_action_lock = threading.Lock()

        self.recorded_points, self.gripper_events = self.load_recording(csv_file)
        if not self.recorded_points:
            raise RuntimeError(f"No trajectory points loaded from {csv_file}")
        self.start_position = self.recorded_points[0][1]

        self.timer = self.create_timer(0.5, self.update)

    def load_recording(self, filename):
        with open(filename, newline="") as csvfile:
            reader = csv.reader(csvfile)
            header = next(reader)
            is_new_format = len(header) >= 10 and header[1] == "row_type"
            raw_points = []
            gripper_events = []
            t0 = None
            for row in reader:
                if not row:
                    continue
                t_ns = int(row[0])
                if t0 is None:
                    t0 = t_ns
                dt = (t_ns - t0) / 1e9
                if is_new_format:
                    row_type = row[1]
                    if row_type == "joint":
                        positions = [float(j) for j in row[3:10]]
                        raw_points.append((dt, positions))
                    elif row_type == "gripper":
                        gripper_events.append((dt, row[2].strip().lower()))
                else:
                    positions = [float(j) for j in row[1:8]]
                    raw_points.append((dt, positions))

        filtered_points = self.smooth_and_downsample_points(raw_points)
        self.get_logger().info(
            f"Loaded {len(raw_points)} joint points, publishing {len(filtered_points)} smoothed points"
        )
        if gripper_events:
            self.get_logger().info(f"Loaded {len(gripper_events)} gripper event(s)")
        return filtered_points, gripper_events

    def smooth_and_downsample_points(self, raw_points):
        if len(raw_points) <= 2:
            return raw_points

        window = SMOOTHING_WINDOW if SMOOTHING_WINDOW % 2 == 1 else SMOOTHING_WINDOW + 1
        half_window = window // 2

        smoothed = []
        for idx, (dt, _) in enumerate(raw_points):
            start = max(0, idx - half_window)
            end = min(len(raw_points), idx + half_window + 1)
            span = raw_points[start:end]

            avg_positions = []
            for joint_idx in range(len(JOINT_NAMES)):
                avg_positions.append(sum(p[1][joint_idx] for p in span) / len(span))
            smoothed.append((dt, avg_positions))

        filtered = [smoothed[0]]
        last_dt = smoothed[0][0]
        for dt, positions in smoothed[1:]:
            if dt - last_dt >= MIN_POINT_DT:
                filtered.append((dt, positions))
                last_dt = dt

        if filtered[-1][0] != smoothed[-1][0]:
            filtered.append(smoothed[-1])

        return filtered

    def joint_state_callback(self, msg):
        joint_map = dict(zip(msg.name, msg.position))
        missing = [j for j in JOINT_NAMES if j not in joint_map]
        if missing:
            self.get_logger().warn(f"Missing joints in joint state sample: {missing}")
            return
        self.actual_positions = [joint_map[j] for j in JOINT_NAMES]

    def update(self):
        if self.full_sent:
            return

        elapsed = (self.get_clock().now().nanoseconds - self.start_time_ns) / 1e9
        if self.actual_positions is None:
            if elapsed > WAIT_FOR_STATE_TIMEOUT_SEC:
                self.get_logger().error(
                    "Timeout: Never received a complete joint state sample. Expected one of: " + ", ".join(JOINT_STATES_TOPICS)
                )
                self.request_stop()
            else:
                self.get_logger().info("Waiting for current joint state on: " + ", ".join(JOINT_STATES_TOPICS))
            return

        ready_publishers = [pub for pub in self.trajectory_publishers if pub.get_subscription_count() > 0]
        if not ready_publishers:
            if elapsed > WAIT_FOR_CONTROLLER_TIMEOUT_SEC:
                self.get_logger().error(
                    "Timeout: No controller subscriber on any trajectory topic. Expected one of: "
                    + ", ".join(TRAJECTORY_TOPICS)
                )
                self.request_stop()
            else:
                self.get_logger().info(
                    "Waiting for trajectory controller subscriber on: " + ", ".join(TRAJECTORY_TOPICS)
                )
            return

        self.get_logger().info("Publishing blended trajectory from current pose into recording")
        trajectory, time_offset, execution_duration = self.build_execution_trajectory()
        for publisher in ready_publishers:
            publisher.publish(trajectory)
        self.execution_start_time = time.monotonic()
        if self.gripper_events:
            self.schedule_gripper_events(time_offset)
        self.schedule_shutdown(execution_duration, time_offset)
        self.full_sent = True
        self.timer.cancel()

    def build_execution_trajectory(self):
        timed_points = []
        start_error = self.max_joint_error(self.actual_positions, self.start_position)
        blend_time = self.compute_blend_time(start_error)
        current_time = 0.0

        if start_error > TOLERANCE:
            blend_steps = max(MIN_BLEND_STEPS, int(blend_time / MIN_POINT_DT))
            blend_step_dt = max(MIN_POINT_DT, blend_time / blend_steps)
            for step_idx in range(1, blend_steps + 1):
                alpha = step_idx / blend_steps
                positions = [
                    current + alpha * (target - current)
                    for current, target in zip(self.actual_positions, self.start_position)
                ]
                current_time += blend_step_dt
                timed_points.append((current_time, positions))
            self.get_logger().info(
                f"Current pose differs from trajectory start by {start_error:.3f} rad; "
                f"blending over {blend_time:.2f} s"
            )
        else:
            self.get_logger().info("Current pose already near trajectory start; skipping blend-in move")

        previous_recorded_dt = None
        for dt, positions in self.recorded_points:
            if previous_recorded_dt is None:
                delta_dt = MIN_POINT_DT
            else:
                delta_dt = max(dt - previous_recorded_dt, MIN_POINT_DT)
            current_time += delta_dt
            timed_points.append((current_time, positions))
            previous_recorded_dt = dt

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.header.stamp = self.get_clock().now().to_msg()

        previous_time = 0.0
        previous_positions = self.actual_positions
        for idx, (point_time, positions) in enumerate(timed_points):
            point = JointTrajectoryPoint()
            point.positions = positions
            if idx == 0:
                segment_dt = max(point_time, MIN_SEGMENT_DT)
            else:
                segment_dt = max(point_time - previous_time, MIN_SEGMENT_DT)
            point.velocities = [
                (pos - prev_pos) / segment_dt
                for pos, prev_pos in zip(positions, previous_positions)
            ]
            point.time_from_start.sec = int(point_time)
            point.time_from_start.nanosec = int((point_time % 1) * 1e9)
            traj.points.append(point)
            previous_time = point_time
            previous_positions = positions

        return traj, blend_time, previous_time

    def schedule_shutdown(self, execution_duration, time_offset):
        latest_gripper_time = max((event_time for event_time, _ in self.gripper_events), default=0.0)
        total_runtime = max(execution_duration, time_offset + latest_gripper_time) + PLAYBACK_COMPLETION_BUFFER_SEC
        self.get_logger().info(f"Playback will auto-exit in {total_runtime:.2f} s after trajectory start")
        self.execution_complete_timer = self.create_timer(total_runtime, self.finish_execution)

    def request_stop(self):
        self.stop_requested = True

    def finish_execution(self):
        if self.execution_complete_timer is not None:
            self.execution_complete_timer.cancel()
            self.execution_complete_timer = None
        self.get_logger().info("Playback complete; shutting down trajectory player")
        self.request_stop()

    def schedule_gripper_events(self, time_offset):
        for event_time, event_name in self.gripper_events:
            event_delay = time_offset + event_time
            threading.Thread(
                target=self.run_gripper_event_after_delay,
                args=(event_delay, event_name),
                daemon=True,
            ).start()

    def run_gripper_event_after_delay(self, delay_sec, event_name):
        time.sleep(max(0.0, delay_sec))
        is_open = event_name == "open"
        action_candidates = GRIPPER_MOVE_ACTION_CANDIDATES if is_open else GRIPPER_GRASP_ACTION_CANDIDATES
        action_cls = Move if is_open else Grasp

        with self.gripper_action_lock:
            try:
                self.execute_gripper_event(action_cls, action_candidates, event_name)
            except RuntimeError as exc:
                self.get_logger().error(str(exc))

    def execute_gripper_event(self, action_cls, action_candidates, event_name):
        node = rclpy.create_node(f"playback_gripper_client_{event_name}_{int(time.time() * 1000)}")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        client = None
        try:
            action_name, client = self.wait_for_gripper_action(node, action_cls, action_candidates, executor)
            if action_name is None:
                raise RuntimeError(
                    "No gripper action server found for playback. Expected an already-running server on one of: "
                    f"{', '.join(action_candidates)}"
                )

            if action_cls is Move:
                goal_msg = Move.Goal()
                goal_msg.width = GRIPPER_OPEN_WIDTH
                goal_msg.speed = GRIPPER_SPEED
                action_label = "Move"
            else:
                goal_msg = Grasp.Goal()
                goal_msg.width = GRIPPER_CLOSE_WIDTH
                goal_msg.speed = GRIPPER_SPEED
                goal_msg.force = GRIPPER_GRASP_FORCE
                goal_msg.epsilon.inner = GRIPPER_EPSILON_INNER
                goal_msg.epsilon.outer = GRIPPER_EPSILON_OUTER
                action_label = "Grasp"

            self.get_logger().info(
                f"Executing recorded gripper event '{event_name}' via {action_name} ({action_label})"
            )
            goal_future = client.send_goal_async(goal_msg)
            self.spin_until_future_complete(executor, goal_future)
            goal_handle = goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError(f"Recorded gripper event '{event_name}' was not accepted by {action_name}")

            result_future = goal_handle.get_result_async()
            self.spin_until_future_complete(executor, result_future)
            result = result_future.result()
            status = getattr(result, 'status', 'unknown')
            self.get_logger().info(
                f"Recorded gripper event '{event_name}' finished with status {status}"
            )
        finally:
            if client is not None:
                client.destroy()
            executor.remove_node(node)
            node.destroy_node()

    def wait_for_gripper_action(self, node, action_cls, action_candidates, executor):
        deadline = time.monotonic() + GRIPPER_ACTION_WAIT_TIMEOUT_SEC
        while time.monotonic() < deadline:
            for candidate in action_candidates:
                client = ActionClient(node, action_cls, candidate)
                if client.wait_for_server(timeout_sec=0.5):
                    return candidate, client
                client.destroy()
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.1)
        return None, None

    def spin_until_future_complete(self, executor, future, timeout_sec: float = 0.1):
        while rclpy.ok() and not future.done():
            executor.spin_once(timeout_sec=timeout_sec)

    def compute_blend_time(self, max_error):
        if max_error <= TOLERANCE:
            return 0.0
        return min(
            MAX_BLEND_TIME_SEC,
            max(MIN_BLEND_TIME_SEC, max_error / BLEND_SPEED_RAD_PER_SEC),
        )

    def max_joint_error(self, current_positions, target_positions):
        return max(abs(a - b) for a, b in zip(current_positions, target_positions))


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: playback_joint_trajectory_smart.py <trajectory.csv>")
        return

    node = SmartTrajectoryPlayer(sys.argv[1])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.stop_requested:
            executor.spin_once(timeout_sec=0.1)
        print("Executed")
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Shutdown")


if __name__ == "__main__":
    main()
