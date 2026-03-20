# franka_kinesthetic_teaching_GUI

A Tkinter-based GUI for kinesthetic teaching and trajectory playback on the Franka Research 3 (FR3) using ROS 2.

## What this repo does

This repo provides a small desktop GUI for two tasks:
- Teach a motion by moving the robot by hand while joint states are recorded to CSV.
- Replay a recorded trajectory through the FR3 joint trajectory controller.

The GUI launches the underlying ROS 2 processes for recording and playback and shows their console output in one window.

## Repo contents

- `franka_teach_run_gui_v2.py`: Tkinter GUI that launches gravity mode, teach, gripper, and run workflows.
- `record_joint_trajectory.py`: Records joint states to a CSV file.
- `playback_joint_trajectory.py`: Loads a CSV, smooths it for playback, blends from the current robot pose, republishes the joint trajectory, and replays recorded gripper events.
- `run_gui.sh`: Convenience launcher for the GUI.

## Features

### Teach mode
- Can launch the gravity compensation example controller automatically if it is not already running.
- Records joint motion while the arm is moved by hand.
- Records GUI gripper button presses as timestamped `open` and `close` events and appends them to the CSV when teach mode stops.
- Prompts for an optional custom CSV filename before recording starts.
- Falls back to the default timestamped CSV name if no custom name is provided.

### Gravity mode
- Provides a dedicated button to start or stop gravity compensation independently from recording.
- Lets you put the robot into gravity compensation without starting the recorder.
- Does not keep a persistent standalone gripper node alive by itself.

### GUI gripper buttons
- `Open Gripper` and `Close Gripper` are one-shot commands.
- Each button press starts the gripper ROS node, sends the command, and then shuts that node down again.
- This is intended to release the gripper connection after each manual command so Franka Desk can regain control.

### Run mode
- Lets you choose a saved CSV trajectory from the GUI.
- Launches the FR3 MoveIt/controller stack.
- Waits for joint state feedback and a trajectory-controller subscriber before publishing.
- Smooths recorded waypoints during playback to reduce vibration.
- Blends from the robot's current joint pose into the recorded trajectory start instead of issuing a separate hard jump to the first point.
- Replays recorded gripper events from the CSV during trajectory execution when matching action servers are available.

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

Playback-specific constants such as smoothing, blend timing, and gripper action names live in [`playback_joint_trajectory.py`](/home/parc/franka_kinesthetic_teaching_GUI/playback_joint_trajectory.py).

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

1. Optionally click `Start Gravity Mode` if you want gravity compensation without recording yet.
2. Click `Start Teach (Record)`.
3. Optionally enter a custom CSV filename, or leave the prompt blank to use the default timestamped name.
4. Wait for gravity compensation to come up if it is not already active.
5. Move the arm by hand to demonstrate the motion.
6. Optionally use `Open Gripper` or `Close Gripper` during teaching if you want those actions captured in the recording.
7. Click `Stop Teach (Save)`.
8. The recording is written in the repo directory using either your custom name or a default name like `joint_trajectory_YYYYMMDD_HHMMSS.csv`.

### Play back a trajectory

1. Click `Run Trajectory`.
2. Choose a previously recorded CSV.
3. The GUI launches the FR3 MoveIt/controller stack.
4. The playback node waits for valid joint states and for the FR3 joint trajectory controller to subscribe.
5. The playback node publishes one full trajectory that:
   - starts from the robot's current joint pose,
   - blends into the recorded start pose,
   - then follows the smoothed recorded trajectory.
6. If the CSV contains `gripper` rows, playback schedules those `open` and `close` events during the replay.

## Recorded CSV format

The recorder writes a mixed event format with this header:

`timestamp_ns,row_type,event,joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,joint_7`

Joint samples are stored as:
- `row_type = joint`
- `event =` empty
- `joint_1` through `joint_7` filled with recorded joint positions

Recorded GUI gripper actions are stored as:
- `row_type = gripper`
- `event = open` or `close`
- Joint columns left empty

The file is written exactly as recorded. Playback smoothing is applied in memory at replay time and does not modify the CSV.

If no filename is provided when recording starts, the recorder saves to the default format:
- `joint_trajectory_YYYYMMDD_HHMMSS.csv`

## Playback smoothing and blending

The current playback behavior is implemented in `playback_joint_trajectory.py`.

### Smoothing behavior
- A moving average is applied across neighboring samples with `SMOOTHING_WINDOW = 5`.
- Waypoints closer together than `MIN_POINT_DT = 0.03` seconds are skipped during playback.
- The replayed trajectory computes waypoint velocities from neighboring points before publishing to the controller.

### Startup behavior
- Playback does not send a separate one-point “move to start” command anymore.
- Instead, it waits for a complete joint-state message and an active subscriber on the trajectory topic.
- It then generates a blended prefix from the robot's current pose to the recorded start pose.
- This avoids the previous failure mode where playback could get stuck waiting at the start.
- This also avoids a hard jump from the previous final pose back to the first recorded waypoint.

### Gripper playback behavior
- Playback reads `gripper` rows from the CSV and schedules them relative to the replayed trajectory timing.
- `open` events are sent through the Franka gripper `Move` action.
- `close` events are sent through the Franka gripper `Grasp` action.
- Playback expects the MoveIt launch to have already started the gripper action server.

## Topics used by this repo

### Playback
- Publishes: `/NS_1/fr3_arm_controller/joint_trajectory` and `/fr3_arm_controller/joint_trajectory`
- Subscribes: `/NS_1/joint_states` and `/joint_states`

### Recording
- Subscribes: `/NS_1/joint_states`

### Manual GUI gripper commands
- Use the Franka gripper ROS action servers under `/<namespace>/franka_gripper/...` when available.

Note: if your FR3 setup publishes joint states or gripper actions on different topics or namespaces, update the scripts accordingly.

## Known assumptions and caveats

- The playback script expects the FR3 joint names `fr3_joint1` through `fr3_joint7`.
- The recorder currently stores `msg.position` as received; it assumes the incoming joint order matches the robot joints you want to replay.
- The GUI uses fixed startup delays for some launched processes. On slow systems, ROS stack startup may still take longer.
- CSV files are saved into the repo working directory by default unless you pass a path as the custom recording name.
- The teach button's filename prompt is optional; leaving it blank keeps the timestamp-based behavior.
- Playback can still trigger a Franka reflex stop such as `power_limit_violation` if the current pose is too far from the recorded start, the path is too aggressive, or the robot is under load.
- If playback aborts at the arm controller level, later gripper events in the same run will not execute because the launched stack is already shutting down.

## Troubleshooting

### Playback starts but the robot does not move
- Check the GUI log for `smart_trajectory_player` timeout messages.
- Check whether the joint-state topics contain all seven FR3 joint names.
- Check whether `fr3_arm_controller` is active and subscribed to the trajectory topic.

### Playback is jerky or vibrates
- Verify you are using the current `playback_joint_trajectory.py`.
- Confirm the controller is accepting the trajectory without timestamp errors.
- Very noisy demonstrations can still produce rough motion even after smoothing.

### Playback aborts with `power_limit_violation`
- This is a Franka reflex stop, not just a ROS warning.
- It means the commanded motion exceeded the robot's allowed power envelope.
- Common causes are a fast blend into the trajectory start, a recorded path with abrupt segments, or trying to replay from a noticeably different starting pose.
- Move the robot closer to the recorded starting posture before playback and keep demonstrations gentle and smooth.

### Franka Desk shows the gripper as busy after using the GUI buttons
- The current button flow is designed to release the gripper connection after each manual `Open` or `Close` command.
- If Desk still shows the end effector as occupied, wait a moment for the gripper node to exit and then use `Kill` in the GUI if needed.

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
