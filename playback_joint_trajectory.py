import csv
import json
import os
import sys
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from franka_msgs.msg import FrankaRobotState
from franka_msgs.action import Grasp, Move
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    "fr3_joint1", "fr3_joint2", "fr3_joint3",
    "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
]
TOLERANCE = 0.05  # rad
START_BLEND_EPSILON_RAD = 0.005
WAIT_FOR_STATE_TIMEOUT_SEC = 15
WAIT_FOR_CONTROLLER_TIMEOUT_SEC = 20
PLAYBACK_COMPLETION_BUFFER_SEC = 0.05
PAUSE_POLL_SEC = 0.05
GRIPPER_EVENT_SETTLE_SEC = 0.75
INITIAL_SETTLE_SEC = 0.10
SMOOTHING_WINDOW = 3  # odd number of samples for moving-average smoothing
MIN_POINT_DT = 0.02  # s, enforce minimum spacing to avoid very abrupt setpoint jumps
MAX_SMOOTHING_DEVIATION_RAD = 0.002  # keep replay very close to the taught path
CURVATURE_PRESERVE_THRESHOLD_RAD = 0.008  # reduce smoothing around tight local features
MIN_BLEND_TIME_SEC = 1.5
MAX_BLEND_TIME_SEC = 12.0
BLEND_SPEED_RAD_PER_SEC = 0.20
FAR_START_ERROR_RAD = 0.35
FAR_START_SLOWDOWN_GAIN = 2.5
MIN_BLEND_STEPS = 10
MIN_SEGMENT_DT = 1e-3
GRIPPER_OPEN_WIDTH = 0.08
GRIPPER_CLOSE_WIDTH = 0.0
GRIPPER_SPEED = 0.1
GRIPPER_GRASP_FORCE = 40.0
GRIPPER_EPSILON_INNER = 0.005
GRIPPER_EPSILON_OUTER = 0.08
REPLAY_LIMIT_MARGIN_RAD = 0.06
ROS_SETUP = "source /opt/ros/humble/setup.bash; source ~/franka_ws/install/setup.bash"
ROBOT_IP = "172.16.0.2"
TEACH_NAMESPACE = "NS_1"
NAMESPACE_PREFIX = f"/{TEACH_NAMESPACE}" if TEACH_NAMESPACE else ""
JOINT_STATES_TOPICS = ["/joint_states"]
TRAJECTORY_TOPICS = ["/fr3_arm_controller/joint_trajectory"]
TRAJECTORY_ACTIONS = ["/fr3_arm_controller/follow_joint_trajectory"]
FRANKA_STATE_TOPICS = ["/franka_robot_state_broadcaster/robot_state"]
if NAMESPACE_PREFIX:
    JOINT_STATES_TOPICS = [
        f"{NAMESPACE_PREFIX}/franka/joint_states",
        f"{NAMESPACE_PREFIX}/joint_states",
        *JOINT_STATES_TOPICS,
    ]
    TRAJECTORY_TOPICS.insert(0, f"{NAMESPACE_PREFIX}/fr3_arm_controller/joint_trajectory")
    TRAJECTORY_ACTIONS.insert(0, f"{NAMESPACE_PREFIX}/fr3_arm_controller/follow_joint_trajectory")
    FRANKA_STATE_TOPICS.insert(0, f"{NAMESPACE_PREFIX}/franka_robot_state_broadcaster/robot_state")
TOPIC_PRIORITY = {topic: index for index, topic in enumerate(JOINT_STATES_TOPICS)}
PREFERRED_JOINT_STATE_TOPIC = JOINT_STATES_TOPICS[0]
PREFERRED_TOPIC_GRACE_SEC = 2.0
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
AUTO_PAUSE_ON_CONTACT = True
AUTO_PAUSE_ON_COLLISION = True
AUTO_PAUSE_EXT_TORQUE_THRESHOLD_NM = 5.0
AUTO_PAUSE_DEBOUNCE_SEC = 0.10
LOWER_JOINT_LIMITS = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]
UPPER_JOINT_LIMITS = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]


class SmartTrajectoryPlayer(Node):
    def __init__(self, csv_file, control_file=None):
        super().__init__("smart_trajectory_player")
        self.csv_file = csv_file
        self.control_file = control_file
        self.trajectory_publishers = [
            self.create_publisher(JointTrajectory, topic, 10)
            for topic in TRAJECTORY_TOPICS
        ]
        self.joint_state_subscriptions = []
        for topic in JOINT_STATES_TOPICS:
            self.joint_state_subscriptions.append(
                self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, source_topic=topic: self.joint_state_callback(msg, source_topic),
                    10,
                )
            )
        self.franka_state_subscriptions = []
        for topic in FRANKA_STATE_TOPICS:
            self.franka_state_subscriptions.append(
                self.create_subscription(
                    FrankaRobotState,
                    topic,
                    lambda msg, source_topic=topic: self.franka_state_callback(msg, source_topic),
                    10,
                )
            )

        self.actual_positions = None
        self.active_joint_topic = None
        self.latest_franka_state = None
        self.active_franka_state_topic = None
        self.first_joint_state_time = None
        self.preferred_joint_state_seen = False
        self.full_sent = False
        self.start_time_ns = self.get_clock().now().nanoseconds
        self.execution_start_time = None
        self.execution_complete_timer = None
        self.stop_requested = False
        self.pause_requested = False
        self.last_control_seq = None
        self.gripper_action_lock = threading.Lock()
        self.trajectory_action_lock = threading.Lock()
        self.playback_thread = None
        self.limit_adjustment_counts = [0] * len(JOINT_NAMES)
        self.current_goal_handle = None
        self.auto_pause_active = False
        self.auto_pause_reason = None
        self.threshold_violation_since = None

        self.recorded_points, self.gripper_events = self.load_recording(csv_file)
        if not self.recorded_points:
            raise RuntimeError(f"No trajectory points loaded from {csv_file}")
        self.start_position = self.recorded_points[0][1]
        self.last_recorded_time = self.recorded_points[-1][0]

        self.timer = self.create_timer(0.5, self.update)

    def load_recording(self, filename):
        with open(filename, newline="") as csvfile:
            reader = csv.reader(csvfile)
            header = next(reader)
            is_new_format = len(header) >= 10 and header[1] == "row_type"
            header_map = {column_name: index for index, column_name in enumerate(header)}
            has_named_arm_columns = all(joint_name in header_map for joint_name in JOINT_NAMES)
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
                        if has_named_arm_columns:
                            positions = [float(row[header_map[joint_name]]) for joint_name in JOINT_NAMES]
                        else:
                            positions = [float(j) for j in row[3:10]]
                        raw_points.append((dt, positions))
                    elif row_type == "gripper":
                        gripper_events.append((dt, row[2].strip().lower()))
                else:
                    positions = [float(j) for j in row[1:8]]
                    raw_points.append((dt, positions))

        filtered_points = self.smooth_and_downsample_points(raw_points)
        filtered_points = self.apply_replay_limit_margin(filtered_points)
        self.get_logger().info(
            f"Loaded {len(raw_points)} joint points, publishing {len(filtered_points)} smoothed points"
        )
        if gripper_events:
            self.get_logger().info(f"Loaded {len(gripper_events)} gripper event(s)")
        self.log_limit_adjustments()
        return filtered_points, gripper_events

    def smooth_and_downsample_points(self, raw_points):
        if len(raw_points) <= 2:
            return raw_points

        window = SMOOTHING_WINDOW if SMOOTHING_WINDOW % 2 == 1 else SMOOTHING_WINDOW + 1
        half_window = window // 2

        smoothed = []
        for idx, (dt, original_positions) in enumerate(raw_points):
            start = max(0, idx - half_window)
            end = min(len(raw_points), idx + half_window + 1)
            span = raw_points[start:end]
            curvature = self.local_path_curvature(raw_points, idx)

            avg_positions = []
            for joint_idx in range(len(JOINT_NAMES)):
                if curvature >= CURVATURE_PRESERVE_THRESHOLD_RAD:
                    smoothed_position = original_positions[joint_idx]
                else:
                    weighted_sum = 0.0
                    total_weight = 0.0
                    for span_idx, (_, span_positions) in enumerate(span):
                        source_idx = start + span_idx
                        distance_from_center = abs(source_idx - idx)
                        weight = float(half_window + 1 - distance_from_center)
                        weighted_sum += span_positions[joint_idx] * weight
                        total_weight += weight
                    smoothed_position = weighted_sum / total_weight

                lower_bound = original_positions[joint_idx] - MAX_SMOOTHING_DEVIATION_RAD
                upper_bound = original_positions[joint_idx] + MAX_SMOOTHING_DEVIATION_RAD
                avg_positions.append(min(max(smoothed_position, lower_bound), upper_bound))
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

    def apply_replay_limit_margin(self, points):
        adjusted_points = []
        for dt, positions in points:
            adjusted_positions = []
            for joint_idx, position in enumerate(positions):
                lower_bound = LOWER_JOINT_LIMITS[joint_idx] + REPLAY_LIMIT_MARGIN_RAD
                upper_bound = UPPER_JOINT_LIMITS[joint_idx] - REPLAY_LIMIT_MARGIN_RAD
                adjusted_position = min(max(position, lower_bound), upper_bound)
                if adjusted_position != position:
                    self.limit_adjustment_counts[joint_idx] += 1
                adjusted_positions.append(adjusted_position)
            adjusted_points.append((dt, adjusted_positions))
        return adjusted_points

    def log_limit_adjustments(self):
        adjustment_messages = []
        for joint_idx, adjustment_count in enumerate(self.limit_adjustment_counts):
            if adjustment_count > 0:
                adjustment_messages.append(
                    f"{JOINT_NAMES[joint_idx]}: {adjustment_count} point(s)"
                )
        if adjustment_messages:
            self.get_logger().warn(
                "Replay limit margin adjusted trajectory points inward for safety: "
                + ", ".join(adjustment_messages)
            )

    def local_path_curvature(self, points, idx):
        if idx <= 0 or idx >= len(points) - 1:
            return 0.0

        previous_positions = points[idx - 1][1]
        current_positions = points[idx][1]
        next_positions = points[idx + 1][1]

        return max(
            abs(next_position - 2.0 * current_position + previous_position)
            for previous_position, current_position, next_position in zip(
                previous_positions, current_positions, next_positions
            )
        )

    def joint_state_callback(self, msg, source_topic):
        joint_map = dict(zip(msg.name, msg.position))
        missing = [j for j in JOINT_NAMES if j not in joint_map]
        if missing:
            return
        if self.first_joint_state_time is None:
            self.first_joint_state_time = time.monotonic()
        if source_topic == PREFERRED_JOINT_STATE_TOPIC:
            self.preferred_joint_state_seen = True
        if self.active_joint_topic is None:
            self.active_joint_topic = source_topic
            self.get_logger().info(f"Using joint states from {source_topic}")
        elif TOPIC_PRIORITY[source_topic] < TOPIC_PRIORITY[self.active_joint_topic]:
            self.active_joint_topic = source_topic
            self.get_logger().info(f"Switching joint state source to preferred topic {source_topic}")
        elif source_topic != self.active_joint_topic:
            return
        self.actual_positions = [joint_map[j] for j in JOINT_NAMES]

    def franka_state_callback(self, msg, source_topic):
        self.latest_franka_state = msg
        if self.active_franka_state_topic is None:
            self.active_franka_state_topic = source_topic
            self.get_logger().info(f"Using Franka robot state from {source_topic}")

    def vector3_active(self, vector_msg):
        return bool(getattr(vector_msg, "x", 0.0) or getattr(vector_msg, "y", 0.0) or getattr(vector_msg, "z", 0.0))

    def compute_auto_pause_reason(self):
        state = self.latest_franka_state
        if state is None:
            return None

        indicators = state.collision_indicators
        if AUTO_PAUSE_ON_COLLISION:
            if any(indicators.is_joint_collision):
                return "joint collision indicator became active"
            if (
                self.vector3_active(indicators.is_cartesian_linear_collision)
                or self.vector3_active(indicators.is_cartesian_angular_collision)
            ):
                return "cartesian collision indicator became active"

        if AUTO_PAUSE_ON_CONTACT:
            if any(indicators.is_joint_contact):
                return "joint contact indicator became active"
            if (
                self.vector3_active(indicators.is_cartesian_linear_contact)
                or self.vector3_active(indicators.is_cartesian_angular_contact)
            ):
                return "cartesian contact indicator became active"

        external_torque = getattr(state.tau_ext_hat_filtered, "effort", [])
        if external_torque:
            peak_torque = max(abs(value) for value in external_torque)
            if peak_torque >= AUTO_PAUSE_EXT_TORQUE_THRESHOLD_NM:
                return (
                    f"external joint torque reached {peak_torque:.2f} Nm "
                    f"(threshold {AUTO_PAUSE_EXT_TORQUE_THRESHOLD_NM:.2f} Nm)"
                )

        return None

    def check_auto_pause_condition(self):
        if self.pause_requested or self.stop_requested:
            self.threshold_violation_since = None
            return

        try:
            reason = self.compute_auto_pause_reason()
        except Exception as exc:
            self.threshold_violation_since = None
            self.get_logger().error(f"Auto-pause monitor error: {exc}")
            return
        if reason is None:
            self.threshold_violation_since = None
            return

        now = time.monotonic()
        if self.threshold_violation_since is None:
            self.threshold_violation_since = now
            return

        if now - self.threshold_violation_since < AUTO_PAUSE_DEBOUNCE_SEC:
            return

        self.pause_requested = True
        self.auto_pause_active = True
        self.auto_pause_reason = reason
        self.threshold_violation_since = None
        self.get_logger().warn(f"Playback auto-paused: {reason}")

    def update(self):
        if self.full_sent:
            return

        elapsed = (self.get_clock().now().nanoseconds - self.start_time_ns) / 1e9
        if self.actual_positions is None:
            if elapsed > WAIT_FOR_STATE_TIMEOUT_SEC:
                self.get_logger().error(
                    "Timeout: Never received a complete joint state sample. Expected one of: "
                    + ", ".join(JOINT_STATES_TOPICS)
                )
                self.request_stop()
            else:
                self.get_logger().info("Waiting for current joint state on: " + ", ".join(JOINT_STATES_TOPICS))
            return

        if (
            self.active_joint_topic != PREFERRED_JOINT_STATE_TOPIC
            and not self.preferred_joint_state_seen
            and self.first_joint_state_time is not None
        ):
            fallback_elapsed = time.monotonic() - self.first_joint_state_time
            if fallback_elapsed < PREFERRED_TOPIC_GRACE_SEC:
                self.get_logger().info(
                    f"Waiting briefly for preferred joint states on {PREFERRED_JOINT_STATE_TOPIC} "
                    f"before starting from fallback topic {self.active_joint_topic}"
                )
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

        self.get_logger().info("Starting segmented playback from current pose into recording")
        self.execution_start_time = time.monotonic()
        self.full_sent = True
        self.timer.cancel()
        self.playback_thread = threading.Thread(
            target=self.run_segmented_playback,
            args=(ready_publishers,),
            daemon=True,
        )
        self.playback_thread.start()

    def build_playback_timeline(self, start_positions, playback_points, gripper_events):
        if not playback_points:
            return {
                "blend_time": 0.0,
                "segments": [],
                "total_duration": 0.0,
            }

        start_position = playback_points[0][1]
        start_error = self.max_joint_error(start_positions, start_position)
        blend_time = self.compute_blend_time(start_error)
        current_time = 0.0
        blend_points = []

        if start_error > START_BLEND_EPSILON_RAD:
            blend_steps = max(MIN_BLEND_STEPS, int(blend_time / MIN_POINT_DT))
            blend_step_dt = max(MIN_POINT_DT, blend_time / blend_steps)
            for step_idx in range(1, blend_steps + 1):
                alpha = step_idx / blend_steps
                # Smoothstep gives a zero-velocity start/end to reduce startup twitch.
                eased_alpha = alpha * alpha * (3.0 - 2.0 * alpha)
                positions = [
                    current + eased_alpha * (target - current)
                    for current, target in zip(start_positions, start_position)
                ]
                current_time += blend_step_dt
                blend_points.append((current_time, positions, False))
            self.get_logger().info(
                f"Current pose differs from trajectory start by {start_error:.3f} rad; "
                f"blending over {blend_time:.2f} s"
            )
        else:
            self.get_logger().info("Current pose already near trajectory start; skipping blend-in move")
            current_time = INITIAL_SETTLE_SEC
            blend_points.append((current_time, list(start_position), False))

        event_entries = [
            {"type": "gripper", "time": event_time, "event_name": event_name}
            for event_time, event_name in gripper_events
        ]
        event_entries.sort(key=lambda item: item["time"])

        segments = []
        previous_recorded_dt = None
        event_index = 0
        current_segment_points = list(blend_points)

        for dt, positions in playback_points:
            while event_index < len(event_entries) and dt >= event_entries[event_index]["time"]:
                if current_segment_points:
                    segment_start = current_segment_points[0][0]
                    segments.append({
                        "type": "trajectory",
                        "absolute_points": current_segment_points,
                        "timed_points": [(t - segment_start, p) for t, p, _ in current_segment_points],
                        "recorded_timed_points": [
                            (t - segment_start, p)
                            for t, p, is_recorded in current_segment_points
                            if is_recorded
                        ],
                        "duration": current_segment_points[-1][0] - segment_start,
                        "recorded_count": sum(1 for _, _, is_recorded in current_segment_points if is_recorded),
                    })
                    current_segment_points = []
                segments.append(event_entries[event_index])
                event_index += 1

            if previous_recorded_dt is None:
                delta_dt = MIN_POINT_DT
            else:
                delta_dt = max(dt - previous_recorded_dt, MIN_POINT_DT)
            current_time += delta_dt
            current_segment_points.append((current_time, positions, True))
            previous_recorded_dt = dt

        if current_segment_points:
            segment_start = current_segment_points[0][0]
            segments.append({
                "type": "trajectory",
                "absolute_points": current_segment_points,
                "timed_points": [(t - segment_start, p) for t, p, _ in current_segment_points],
                "recorded_timed_points": [
                    (t - segment_start, p)
                    for t, p, is_recorded in current_segment_points
                    if is_recorded
                ],
                "duration": current_segment_points[-1][0] - segment_start,
                "recorded_count": sum(1 for _, _, is_recorded in current_segment_points if is_recorded),
            })

        while event_index < len(event_entries):
            segments.append(event_entries[event_index])
            event_index += 1

        return {
            "blend_time": blend_time,
            "segments": segments,
            "total_duration": current_time,
        }

    def build_trajectory_message(self, timed_points, start_positions):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.header.stamp = self.get_clock().now().to_msg()

        previous_time = 0.0
        previous_positions = start_positions

        for point_time, positions in timed_points:
            point = JointTrajectoryPoint()
            point.positions = positions
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

        if traj.points:
            # Ensure the controller sees a zero terminal velocity at the end of each segment.
            traj.points[-1].velocities = [0.0] * len(JOINT_NAMES)

        return traj

    def publish_trajectory_segment(self, ready_publishers, timed_points, start_positions):
        if not timed_points:
            return 0.0
        traj = self.build_trajectory_message(timed_points, start_positions)
        for publisher in ready_publishers:
            publisher.publish(traj)
        return timed_points[-1][0]

    def publish_hold_position(self, ready_publishers, hold_positions, hold_duration=0.25):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.header.stamp = self.get_clock().now().to_msg()

        point = JointTrajectoryPoint()
        point.positions = list(hold_positions)
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start.sec = int(hold_duration)
        point.time_from_start.nanosec = int((hold_duration % 1) * 1e9)
        traj.points.append(point)

        for publisher in ready_publishers:
            publisher.publish(traj)

    def wait_for_trajectory_action(self):
        action_clients = []
        try:
            for action_name in TRAJECTORY_ACTIONS:
                client = ActionClient(self, FollowJointTrajectory, action_name)
                action_clients.append(client)
                if client.wait_for_server(timeout_sec=0.5):
                    return action_name, client
        finally:
            pass
        for client in action_clients:
            client.destroy()
        return None, None

    def execute_trajectory_segment_action(self, timed_points, start_positions, recorded_timed_points):
        if not timed_points:
            return "completed", timed_points[-1][1] if timed_points else start_positions, 0

        action_name, client = self.wait_for_trajectory_action()
        if action_name is None:
            raise RuntimeError(
                "No trajectory action server found. Expected one of: " + ", ".join(TRAJECTORY_ACTIONS)
            )
        try:
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = self.build_trajectory_message(timed_points, start_positions)
            self.get_logger().info(
                f"Executing continuous trajectory segment with {len(timed_points)} point(s) over {timed_points[-1][0]:.2f} s via {action_name}"
            )
            send_future = client.send_goal_async(goal_msg)
            while rclpy.ok() and not send_future.done():
                self.update_control_state()
                self.check_auto_pause_condition()
                if self.stop_requested:
                    return "stopped", start_positions, 0
                time.sleep(PAUSE_POLL_SEC)

            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError(f"Trajectory segment goal was not accepted by {action_name}")

            self.current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            goal_start_time = time.monotonic()
            while rclpy.ok() and not result_future.done():
                self.update_control_state()
                self.check_auto_pause_condition()
                if self.pause_requested:
                    elapsed_time = time.monotonic() - goal_start_time
                    consumed_points = self.estimate_consumed_recorded_points(
                        recorded_timed_points,
                        self.actual_positions,
                        elapsed_time,
                    )
                    cancel_future = goal_handle.cancel_goal_async()
                    while rclpy.ok() and not cancel_future.done():
                        time.sleep(PAUSE_POLL_SEC)
                    self.current_goal_handle = None
                    return "paused", self.actual_positions[:] if self.actual_positions is not None else start_positions, consumed_points
                if self.stop_requested:
                    cancel_future = goal_handle.cancel_goal_async()
                    while rclpy.ok() and not cancel_future.done():
                        time.sleep(PAUSE_POLL_SEC)
                    self.current_goal_handle = None
                    return "stopped", self.actual_positions[:] if self.actual_positions is not None else start_positions, 0
                time.sleep(PAUSE_POLL_SEC)

            self.current_goal_handle = None
            result = result_future.result()
            status = getattr(result, "status", None)
            if status != 4:
                raise RuntimeError(f"Trajectory segment action finished with status {status}")
            return "completed", timed_points[-1][1], len(recorded_timed_points)
        finally:
            client.destroy()

    def estimate_consumed_recorded_points(self, recorded_timed_points, current_positions, elapsed_time):
        if not recorded_timed_points or current_positions is None:
            return 0

        if elapsed_time <= 0.0:
            candidate_limit = 1
        else:
            candidate_limit = 0
            for point_time, _ in recorded_timed_points:
                if point_time <= elapsed_time + PLAYBACK_COMPLETION_BUFFER_SEC:
                    candidate_limit += 1
            candidate_limit = max(1, candidate_limit)

        best_index = 0
        best_error = None
        search_points = recorded_timed_points[:candidate_limit]
        for index, (_, point_positions) in enumerate(search_points, start=1):
            error = self.max_joint_error(current_positions, point_positions)
            if best_error is None or error < best_error:
                best_error = error
                best_index = index
        return best_index

    def update_control_state(self):
        if not self.control_file or not os.path.exists(self.control_file):
            return
        try:
            with open(self.control_file, encoding="utf-8") as handle:
                payload = json.load(handle)
        except (OSError, json.JSONDecodeError):
            return

        seq = payload.get("seq")
        if seq == self.last_control_seq:
            return

        self.last_control_seq = seq
        command = payload.get("command")
        if command == "pause":
            self.pause_requested = True
            self.get_logger().info("Pause requested by GUI")
        elif command == "resume":
            self.pause_requested = False
            self.auto_pause_active = False
            self.auto_pause_reason = None
            self.threshold_violation_since = None
            self.get_logger().info("Resume requested by GUI")
        elif command == "stop":
            self.get_logger().info("Stop requested by GUI")
            self.request_stop()

    def sleep_with_control(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.stop_requested:
                return False
            self.update_control_state()
            if self.pause_requested:
                return False
            time.sleep(min(PAUSE_POLL_SEC, max(0.0, deadline - time.monotonic())))
        return True

    def wait_until_resumed(self, ready_publishers):
        hold_published = False
        while not self.stop_requested and self.pause_requested:
            if self.actual_positions is not None and not hold_published:
                self.publish_hold_position(ready_publishers, self.actual_positions)
                if self.auto_pause_active and self.auto_pause_reason:
                    self.get_logger().warn(
                        f"Playback paused automatically; holding current pose until resume ({self.auto_pause_reason})"
                    )
                else:
                    self.get_logger().info("Playback paused; holding current pose until resume")
                hold_published = True
            self.update_control_state()
            time.sleep(PAUSE_POLL_SEC)

        if not self.stop_requested and hold_published:
            if self.auto_pause_active:
                self.get_logger().info("Playback resumed after auto-pause; rebuilding remaining trajectory from current pose")
            else:
                self.get_logger().info("Playback resumed; rebuilding remaining trajectory from current pose")
            self.auto_pause_active = False
            self.auto_pause_reason = None
            self.threshold_violation_since = None

    def run_segmented_playback(self, ready_publishers):
        try:
            current_positions = self.actual_positions[:]
            current_point_index = 0
            next_event_index = 0
            total_runtime = 0.0

            while current_point_index < len(self.recorded_points) or next_event_index < len(self.gripper_events):
                self.update_control_state()
                if self.stop_requested:
                    return
                if self.pause_requested:
                    self.wait_until_resumed(ready_publishers)
                    if self.stop_requested:
                        return
                    if self.actual_positions is not None:
                        current_positions = self.actual_positions[:]
                    continue

                remaining_points = self.recorded_points[current_point_index:]
                remaining_events = self.gripper_events[next_event_index:]
                timeline = self.build_playback_timeline(current_positions, remaining_points, remaining_events)

                for entry in timeline["segments"]:
                    if self.stop_requested:
                        return
                    if self.pause_requested:
                        break
                    if entry["type"] == "trajectory":
                        duration = entry["timed_points"][-1][0] if entry["timed_points"] else 0.0
                        status, end_positions, consumed_points = self.execute_trajectory_segment_action(
                            entry["timed_points"],
                            current_positions,
                            entry["recorded_timed_points"],
                        )
                        if status == "completed":
                            total_runtime += duration + PLAYBACK_COMPLETION_BUFFER_SEC
                            current_positions = end_positions
                            current_point_index += consumed_points
                            if not self.sleep_with_control(PLAYBACK_COMPLETION_BUFFER_SEC):
                                break
                        elif status == "paused":
                            if end_positions is not None:
                                current_positions = end_positions
                            current_point_index += consumed_points
                            break
                        else:
                            return
                    elif entry["type"] == "gripper":
                        self.get_logger().info(
                            f"Holding arm trajectory for recorded gripper event '{entry['event_name']}' at {entry['time']:.2f} s"
                        )
                        self.execute_timed_gripper_event(entry["event_name"])
                        next_event_index += 1
                        if not self.sleep_with_control(GRIPPER_EVENT_SETTLE_SEC):
                            break
                        total_runtime += GRIPPER_EVENT_SETTLE_SEC

                if self.stop_requested:
                    return
                if self.pause_requested:
                    continue
                if self.actual_positions is not None:
                    current_positions = self.actual_positions[:]

            self.get_logger().info(
                f"Segmented playback finished in {total_runtime:.2f} s; shutting down trajectory player"
            )
            self.request_stop()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            self.request_stop()

    def execute_timed_gripper_event(self, event_name):
        is_open = event_name == "open"
        action_candidates = GRIPPER_MOVE_ACTION_CANDIDATES if is_open else GRIPPER_GRASP_ACTION_CANDIDATES
        action_cls = Move if is_open else Grasp
        with self.gripper_action_lock:
            self.execute_gripper_event(action_cls, action_candidates, event_name)

    def request_stop(self):
        self.stop_requested = True

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
            status = getattr(result, "status", "unknown")
            if status != 4:
                raise RuntimeError(
                    f"Recorded gripper event '{event_name}' failed via {action_name} with status {status}"
                )
            self.get_logger().info(f"Recorded gripper event '{event_name}' finished with status {status}")
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

    def spin_until_future_complete(self, executor, future, timeout_sec=0.1):
        while rclpy.ok() and not future.done():
            executor.spin_once(timeout_sec=timeout_sec)

    def compute_blend_time(self, max_error):
        if max_error <= START_BLEND_EPSILON_RAD:
            return 0.0
        base_time = max(MIN_BLEND_TIME_SEC, max_error / BLEND_SPEED_RAD_PER_SEC)
        if max_error <= FAR_START_ERROR_RAD:
            return min(MAX_BLEND_TIME_SEC, base_time)

        far_error = max_error - FAR_START_ERROR_RAD
        slowdown_scale = 1.0 + FAR_START_SLOWDOWN_GAIN * (far_error / FAR_START_ERROR_RAD) ** 2
        return min(MAX_BLEND_TIME_SEC, base_time * slowdown_scale)

    def max_joint_error(self, current_positions, target_positions):
        return max(abs(a - b) for a, b in zip(current_positions, target_positions))


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: playback_joint_trajectory_smart.py <trajectory.csv> [control.json]")
        return

    control_file = sys.argv[2] if len(sys.argv) >= 3 else None
    node = SmartTrajectoryPlayer(sys.argv[1], control_file)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.stop_requested:
            executor.spin_once(timeout_sec=0.1)
        print("Executed")
    except ExternalShutdownException:
        pass
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
