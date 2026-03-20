# franka_kinesthetic_teaching_GUI

A Tkinter-based GUI for kinesthetic teaching and trajectory playback on the Franka Research 3 (FR3) using ROS 2.

## What this repo does

This repo provides a small desktop GUI for two tasks:
- Teach a motion by moving the robot by hand while joint states are recorded to CSV.
- Replay a recorded trajectory through the FR3 joint trajectory controller.

The GUI launches the underlying ROS 2 processes for recording and playback and shows their console output in one window.

## Repo contents

- `franka_teach_run_gui_v2.py`: Tkinter GUI that launches teach and run workflows.
- `record_joint_trajectory.py`: Records joint states to a timestamped CSV file.
- `playback_joint_trajectory.py`: Loads a CSV, smooths it for playback, blends from the current robot pose, and publishes a joint trajectory.
- `run_gui.sh`: Convenience launcher for the GUI.

## Features

### Teach mode
- Launches the gravity compensation example controller.
- Records joint motion while the arm is moved by hand.
- Saves a timestamped CSV when recording is stopped.

### Run mode
- Lets you choose a saved CSV trajectory from the GUI.
- Launches the FR3 MoveIt/controller stack.
- Waits for joint state feedback and a trajectory-controller subscriber before publishing.
- Smooths recorded waypoints during playback to reduce vibration.
- Blends from the robot's current joint pose into the recorded trajectory start instead of issuing a separate hard jump to the first point.

## Requirements

- ROS 2 Humble
- `franka_ros2` packages installed and configured
- Python 3
- Tkinter for Python: `sudo apt install python3-tk`
- A working FR3 setup with FCI enabled and the robot reachable on your network

## Configuration

The main user-editable settings are in [`franka_teach_run_gui_v2.py`](/home/parc/franka_kinesthetic_teaching_GUI/franka_teach_run_gui_v2.py):

- `ROS_SETUP`: shell command used to source your ROS environment
- `ROBOT_IP`: IP address passed to the FR3 launch command

Update those values for your machine before running the GUI.

## Running

1. Make sure the robot is powered, reachable, and joints are unlocked.
2. Make sure FCI mode is active.
3. Open a terminal.
4. Change into this repo.
5. Start the GUI.

```bash
cd ~/franka_kinesthetic_teaching_GUI
./run_gui.sh
```

## Usage

### Record a trajectory

1. Click `Start Teach (Record)`.
2. Wait for gravity compensation to come up.
3. Move the arm by hand to demonstrate the motion.
4. Click `Stop Teach (Save)`.
5. A CSV named like `joint_trajectory_YYYYMMDD_HHMMSS.csv` is written in the repo directory.

### Play back a trajectory

1. Click `Run Trajectory`.
2. Choose a previously recorded CSV.
3. The GUI launches the FR3 MoveIt/controller stack.
4. The playback node waits for valid `/joint_states` and for the FR3 joint trajectory controller to subscribe.
5. The playback node publishes one full trajectory that:
   - starts from the robot's current joint pose,
   - blends into the recorded start pose,
   - then follows the smoothed recorded trajectory.

## Recorded CSV format

The recorder stores one row per sample:
- column 1: relative timestamp in nanoseconds from the start of recording
- columns 2-8: joint positions

The file is written exactly as recorded. Playback smoothing is applied in memory at replay time and does not modify the CSV.

## Playback smoothing and blending

The current playback behavior is implemented in `playback_joint_trajectory.py`.

### Smoothing behavior
- A moving average is applied across neighboring samples with `SMOOTHING_WINDOW = 5`.
- Waypoints closer together than `MIN_POINT_DT = 0.03` seconds are skipped during playback.
- The replayed trajectory computes waypoint velocities from neighboring points before publishing to the controller.

### Startup behavior
- Playback does not send a separate one-point “move to start” command anymore.
- Instead, it waits for a complete `/joint_states` message and an active subscriber on `/fr3_arm_controller/joint_trajectory`.
- It then generates a blended prefix from the robot's current pose to the recorded start pose.
- This avoids the previous failure mode where playback could get stuck waiting at the start.
- This also avoids a hard jump from the previous final pose back to the first recorded waypoint.

## Topics used by this repo

### Playback
- Publishes: `/fr3_arm_controller/joint_trajectory`
- Subscribes: `/joint_states`

### Recording
- Subscribes: `/NS_1/joint_states`

Note: recording and playback currently use different joint-state topics. That reflects the current scripts in this repo. If your FR3 setup publishes joint states on different topics or namespaces, update the scripts accordingly.

## Known assumptions and caveats

- The playback script expects the FR3 joint names:
  `fr3_joint1` through `fr3_joint7`.
- The recorder currently stores `msg.position` as received; it assumes the incoming joint order matches the robot joints you want to replay.
- The GUI uses fixed startup delays for some launched processes. On slow systems, ROS stack startup may still take longer.
- CSV files are saved into the repo working directory by default.

## Troubleshooting

### Playback starts but the robot does not move
- Check the GUI log for `smart_trajectory_player` timeout messages.
- Check whether `/joint_states` contains all seven FR3 joint names.
- Check whether `fr3_arm_controller` is active and subscribed to `/fr3_arm_controller/joint_trajectory`.

### Playback is jerky or vibrates
- Verify you are using the current `playback_joint_trajectory.py`.
- Confirm the controller is accepting the trajectory without timestamp errors.
- Very noisy demonstrations can still produce rough motion even after smoothing.

### Recording saves no CSV
- If no joint messages were received during teach mode, the recorder will refuse to save an empty file.
- Check that the recording joint-state topic matches your running system.

## Installation

```bash
git clone https://github.com/ulubilgeulusoy/franka_kinesthetic_teaching_GUI.git
cd franka_kinesthetic_teaching_GUI
```

## Acknowledgements

This project was developed with assistance from Codex by OpenAI.
