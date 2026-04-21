# franka_kinesthetic_teaching_GUI

A small Tkinter desktop application for kinesthetic teaching and trajectory playback on a Franka Research 3 (FR3) with ROS 2.

## Validated environment

This branch is currently set up and maintained for a native Ubuntu 22.04 + ROS 2 Humble workflow.

- OS: Ubuntu 22.04 LTS
- ROS distro: ROS 2 Humble
- Python: Python 3
- Robot: Franka Research 3 (FR3)

Important notes:
- This branch is the Humble-oriented branch and does not track the Jazzy setup from `main`.
- `run_gui.sh` is intended to be the entry point and should source `/opt/ros/humble/setup.bash` and your local Franka workspace.
- A realtime kernel is still recommended for smoother control behavior, even though the GUI can run without perfect realtime tuning.

## What this repo does

This repo contains one main application: a GUI that manages two workflows:
- teach a motion by moving the robot by hand while joint states are recorded to CSV
- replay a recorded trajectory through the FR3 joint trajectory controller

The GUI also provides manual gripper buttons and a dedicated gravity-compensation mode.

## Main application

The main application is [`franka_teach_run_gui_v2.py`](/home/parc/franka_kinesthetic_teaching_GUI/franka_teach_run_gui_v2.py). It is launched by [`run_gui.sh`](/home/parc/franka_kinesthetic_teaching_GUI/run_gui.sh), which sources ROS and then runs the Tkinter app.

The GUI exposes these actions:
- `Start Teach (Record)`
- `Start Gravity Mode`
- `Open Gripper`
- `Close Gripper`
- `Run Trajectory`
- `Kill`

## Repo contents

- `franka_teach_run_gui_v2.py`: Tkinter GUI for teach, gravity, gripper, and playback workflows
- `run_gui.sh`: convenience launcher for the GUI
- `franka_teach_minimal.launch.py`: reduced teach/gravity launch used by the GUI
- `franka_teach.config.yaml`: robot config passed into the minimal teach launch
- `record_joint_trajectory.py`: records joint states to CSV
- `playback_joint_trajectory.py`: loads a CSV, applies bounded smoothing, blends from the current pose, publishes segmented arm trajectories, and replays recorded gripper events

## How the GUI uses the launch files

### Minimal teach launch

[`franka_teach_minimal.launch.py`](/home/parc/franka_kinesthetic_teaching_GUI/franka_teach_minimal.launch.py) is used by the GUI for:
- `Start Teach (Record)`
- `Start Gravity Mode`

It brings up:
- `robot_state_publisher`
- `controller_manager` / `ros2_control_node`
- `joint_state_publisher`
- `joint_state_broadcaster`
- `gravity_compensation_example_controller`
- the Franka gripper launch, if `load_gripper` is enabled

It does not bring up the full MoveIt playback stack.

The teach config currently used by the GUI is [`franka_teach.config.yaml`](/home/parc/franka_kinesthetic_teaching_GUI/franka_teach.config.yaml), which sets:
- namespace `NS_1`
- `arm_id: fr3`
- `robot_ip: 172.16.0.2`
- `load_gripper: "true"`
- `use_fake_hardware: "false"`

### Playback launch

When you click `Run Trajectory`, the GUI launches:
- `ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=... namespace:=NS_1 arm_id:=fr3`

After a short delay, it runs [`playback_joint_trajectory.py`](/home/parc/franka_kinesthetic_teaching_GUI/playback_joint_trajectory.py) on the selected CSV.

## Features

### Teach mode

- Starts the minimal teach bringup automatically if it is not already running
- Records joint motion while the arm is moved by hand
- Prompts for an optional CSV filename before recording starts
- Uses a timestamped filename if you leave the prompt blank
- Records GUI gripper button presses as timestamped `open` and `close` events
- Merges confirmed gripper events into the CSV in timestamp order when teach mode stops
- Uses the already-running gripper action server from the teach/gravity stack
- Avoids a separate gripper connection while arm control is active

### Gravity mode

- Starts the same minimal teach/gravity launch without starting the recorder
- Lets you put the arm into gravity compensation independently from recording
- Exposes the gripper action server from the same bringup stack

### GUI gripper buttons

- `Open Gripper` sends a Franka `Move` action
- `Close Gripper` sends a Franka `Grasp` action
- During teach/gravity mode, the GUI uses the gripper action server from the already-running teach stack
- Outside teach/gravity mode, the GUI starts a standalone `franka_gripper` launch, sends the command, then shuts it down again
- The GUI refuses to start a second gripper node while teach/gravity arm control is active but no gripper action server is available

### Run mode

- Lets you choose a saved CSV from the GUI
- Launches the FR3 MoveIt/controller stack
- Waits for a gripper action server before starting playback
- The playback node waits for joint state feedback and a trajectory-controller subscriber before publishing
- Smooths recorded waypoints during playback
- Blends from the robot's current joint pose into the recorded trajectory start when needed
- Publishes segmented arm trajectories and pauses between segments to execute recorded gripper events
- Waits briefly for the preferred Franka joint-state topic before starting from a fallback topic

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- `franka_ros2` packages installed and configured
- Python 3
- Tkinter for Python: `sudo apt install python3-tk`
- A working FR3 setup with FCI enabled and reachable on your network
- Recommended: a realtime kernel for better runtime behavior

## Configuration

Main user-editable settings live in [`franka_teach_run_gui_v2.py`](/home/parc/franka_kinesthetic_teaching_GUI/franka_teach_run_gui_v2.py):
- `ROS_SETUP`
- `ROBOT_IP`
- `TEACH_NAMESPACE`

Teach/gravity robot settings live in [`franka_teach.config.yaml`](/home/parc/franka_kinesthetic_teaching_GUI/franka_teach.config.yaml).

Playback constants such as smoothing, blend timing, and gripper action candidates live in [`playback_joint_trajectory.py`](/home/parc/franka_kinesthetic_teaching_GUI/playback_joint_trajectory.py).
Recorder topic-selection safeguards live in [`record_joint_trajectory.py`](/home/parc/franka_kinesthetic_teaching_GUI/record_joint_trajectory.py).

## Running

```bash
cd ~/franka_kinesthetic_teaching_GUI
./run_gui.sh
```

## Usage

### Record a trajectory

1. Optionally click `Start Gravity Mode` if you want gravity compensation without recording yet.
2. Click `Start Teach (Record)`.
3. Optionally enter a custom CSV filename, or leave it blank for a timestamped default.
4. Wait for the recorder to start receiving valid Franka joint states.
5. Only then move the arm by hand to demonstrate the motion.
6. Optionally use `Open Gripper` or `Close Gripper` during teaching if you want those actions recorded.
7. Click `Stop Teach (Save)`.

### Play back a trajectory

1. Click `Run Trajectory`.
2. Choose a previously recorded CSV.
3. The GUI launches the FR3 MoveIt/controller stack.
4. The GUI waits for a gripper action server.
5. The playback node waits for valid joint states and an active trajectory-controller subscriber.
6. Playback publishes segmented trajectories that blend from the current pose into the recording when needed.
7. If the CSV contains `gripper` rows, playback pauses between arm segments and executes those recorded `open` and `close` events.

## Recorded CSV format

The recorder writes this header:

`timestamp_ns,row_type,event,fr3_joint1,fr3_joint2,fr3_joint3,fr3_joint4,fr3_joint5,fr3_joint6,fr3_joint7`

Joint samples are stored as:
- `row_type = joint`
- `event =` empty
- `fr3_joint1` through `fr3_joint7` filled in explicit arm-joint-name order

Recorded GUI gripper actions are stored as:
- `row_type = gripper`
- `event = open` or `close`
- joint columns left empty

If you provide no filename when recording starts, the recorder uses:
- `joint_trajectory_YYYYMMDD_HHMMSS.csv`

## Playback behavior

The current playback behavior is implemented in [`playback_joint_trajectory.py`](/home/parc/franka_kinesthetic_teaching_GUI/playback_joint_trajectory.py).

### Smoothing and downsampling

- A light moving average is applied with `SMOOTHING_WINDOW = 3`
- Waypoints closer together than `MIN_POINT_DT = 0.02` seconds are skipped
- Smoothing is bounded so each replayed joint sample stays close to the taught path with `MAX_SMOOTHING_DEVIATION_RAD = 0.002`
- Tight local path features are preserved by reducing smoothing when curvature exceeds `CURVATURE_PRESERVE_THRESHOLD_RAD = 0.008`
- Published trajectory points include velocities computed from neighboring points

### Blend-in behavior

- Playback waits for a complete joint-state sample before publishing
- Playback also waits for at least one subscriber on a trajectory topic
- If the current pose is farther than `TOLERANCE = 0.05` rad from the recorded start, the player inserts a blend-in segment
- If playback only sees a fallback joint-state topic first, it briefly waits for the preferred Franka joint-state topic before starting
- The blend uses a smoothstep easing profile to soften startup motion
- Blend duration is bounded by:
  `MIN_BLEND_TIME_SEC = 1.5`
  `MAX_BLEND_TIME_SEC = 12.0`
  `BLEND_SPEED_RAD_PER_SEC = 0.20`
- Additional far-start slowdown is applied with:
  `FAR_START_ERROR_RAD = 0.35`
  `FAR_START_SLOWDOWN_GAIN = 2.5`
- Very small startup mismatches below `START_BLEND_EPSILON_RAD = 0.005` rad do not trigger a full blend
- When the current pose is already near the recorded start, playback inserts a short `INITIAL_SETTLE_SEC = 0.10` hold before the recorded motion proceeds

### Gripper replay behavior

- Playback reads `gripper` rows from the CSV and inserts them into the replay timeline
- `open` events use the Franka gripper `Move` action
- `close` events use the Franka gripper `Grasp` action
- The arm trajectory is intentionally held while each gripper event executes
- Playback expects an already-running gripper action server, typically from the MoveIt launch started by the GUI

## Topics used by this repo

### Playback

- Publishes: `/NS_1/fr3_arm_controller/joint_trajectory` and `/fr3_arm_controller/joint_trajectory`
- Subscribes, in priority order: `/NS_1/franka/joint_states`, `/NS_1/joint_states`, `/joint_states`

### Recording

- Subscribes, in priority order: `/NS_1/franka/joint_states`, `/NS_1/joint_states`, `/joint_states`
- Prefers `/NS_1/franka/joint_states` and waits before falling back
- Rejects fallback startup samples that only match the default-looking pose `0, 0, 0, -1.59695, 0, 2.5307, 0`

### Manual GUI gripper commands

- Uses Franka gripper action servers under the namespace candidates checked by the GUI and playback scripts:
  `/franka_gripper/...`
  `/NS_1/franka_gripper/...`
  `/fr3_gripper/...`
  `/NS_1/fr3_gripper/...`

## Known assumptions and caveats

- The playback script expects FR3 joint names `fr3_joint1` through `fr3_joint7`
- The GUI uses fixed startup delays in a few places, so slow systems may still need more time
- The run flow currently waits a fixed 3 seconds after starting MoveIt before launching playback, then relies on runtime readiness checks inside the playback node
- CSV files are saved into the repo directory by default unless you provide another path
- Playback can still trigger a Franka reflex stop such as `power_limit_violation` if the current pose is too far from the recording start, the path is too aggressive, or the robot is under load
- If playback aborts at the arm-controller level, later gripper events in the same run will not execute because the launched stack is already shutting down

## Installation

```bash
git clone https://github.com/ulubilgeulusoy/franka_kinesthetic_teaching_GUI.git
cd franka_kinesthetic_teaching_GUI
git checkout Humble_KT
```

## Acknowledgements

This project was developed with assistance from Codex by OpenAI.
