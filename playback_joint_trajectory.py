import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import csv
import sys
import time

JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
    'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
]
TOLERANCE = 0.05  # rad
TIMEOUT_SEC = 15  # Optional: how long to wait for start position before abort
SMOOTHING_WINDOW = 5  # odd number of samples for moving-average smoothing
MIN_POINT_DT = 0.03  # s, enforce minimum spacing to avoid very abrupt setpoint jumps

class SmartTrajectoryPlayer(Node):
    def __init__(self, csv_file):
        super().__init__('smart_trajectory_player')
        self.csv_file = csv_file
        self.publisher_ = self.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.actual_positions = None
        self.start_sent = False
        self.full_sent = False
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]  ### For timeout

        self.trajectory = self.load_trajectory(csv_file)
        self.start_position = self.trajectory.points[0].positions

        self.timer = self.create_timer(0.5, self.update)

    def load_trajectory(self, filename):
        with open(filename, newline='') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # Skip header
            raw_points = []
            t0 = None
            for row in reader:
                t_ns = int(row[0])
                if t0 is None:
                    t0 = t_ns
                dt = (t_ns - t0) / 1e9
                positions = [float(j) for j in row[1:8]]
                raw_points.append((dt, positions))

        filtered_points = self.smooth_and_downsample_points(raw_points)

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        for idx, (dt, positions) in enumerate(filtered_points):
            point = JointTrajectoryPoint()
            point.positions = positions
            if idx == 0:
                point.velocities = [0.0] * len(positions)
            else:
                prev_dt, prev_pos = filtered_points[idx - 1]
                seg_dt = max(dt - prev_dt, 1e-3)
                point.velocities = [(p - pp) / seg_dt for p, pp in zip(positions, prev_pos)]
            point.time_from_start.sec = int(dt)
            point.time_from_start.nanosec = int((dt % 1) * 1e9)
            traj.points.append(point)

        self.get_logger().info(
            f"Loaded {len(raw_points)} points, publishing {len(filtered_points)} smoothed points"
        )
        return traj

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
        # Warn if any joint is missing
        if not all(j in joint_map for j in JOINT_NAMES):
            missing = [j for j in JOINT_NAMES if j not in joint_map]
            self.get_logger().warn(f"Missing joints in /joint_states: {missing}")
        self.actual_positions = [joint_map.get(j, 0.0) for j in JOINT_NAMES]

    def update(self):
        # Timeout check (optional)
        elapsed = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        if not self.start_sent and elapsed > TIMEOUT_SEC:
            self.get_logger().error("Timeout: Never reached start position. Aborting.")
            rclpy.shutdown()
            return

        if not self.start_sent:
            self.get_logger().info("Sending move-to-start position")
            self.send_start_position()
            self.start_sent = True
            return
        if not self.full_sent and self.actual_positions is not None:
            if self.is_at_start_position():
                self.get_logger().info("Reached start position, sending full trajectory")
                self.publisher_.publish(self.trajectory)
                self.full_sent = True
                self.timer.cancel()
            else:
                self.get_logger().info("Waiting to reach start position...")

    def is_at_start_position(self):
        if self.actual_positions is None:
            return False
        errors = [abs(a - d) for a, d in zip(self.actual_positions, self.start_position)]
        return all(e < TOLERANCE for e in errors)

    def send_start_position(self):
        point = JointTrajectoryPoint()
        point.positions = self.start_position
        point.velocities = [0.0] * len(point.positions)
        point.time_from_start.sec = 3

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        traj.points.append(point)

        # Optionally, set a header stamp if your controller is picky
        traj.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: playback_joint_trajectory_smart.py <trajectory.csv>")
        return
    node = SmartTrajectoryPlayer(sys.argv[1])
    try:
        rclpy.spin(node)
        print("Executed")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Shutdown")

if __name__ == '__main__':
    main()
