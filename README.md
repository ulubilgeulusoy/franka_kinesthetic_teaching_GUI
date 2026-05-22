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

### Branch-specific backend

The `Humble_KT_failsafe` branch is wired to the experimental workspace:

- `~/franka_ws_jointfailsafe`

instead of the original backend:

- `~/franka_ws`

This branch is therefore intended for testing the experimental soft-stop kinesthetic backend without modifying the original workspace.

## What this repo does

This repo contains one main application: a GUI that manages two workflows:
- teach a motion by moving the robot by hand while joint states are recorded to CSV
- replay a recorded trajectory through the FR3 joint trajectory controller

- `Start Gravity Mode` can launch the reduced gravity-compensation mode and lets you move the robot arm freely
- `Open Gripper` and `Close Gripper` buttons enables you to operate gripper during teaching, gravity mode, or stadalone
- `Start Teach (Record)` enables you to start teaching, prompt for a CSV filename or don't, start the recorder, when you finish the teaching press the `Stop Teach (Save)` button.
- `Run Trajectory` enables you to select the recorded trajectory to run it, but only when the robot is in a clean run-ready state; it launches the MoveIt/controller playback stack, checks readiness, then shuts it down again when playback finishes

In practice, this repo is meant to be the operator-facing front end for day-to-day kinesthetic teaching on the FR3.

## Robot-State / LSL Integration

This GUI now publishes explicit kinesthetic mode state to the local FR3 robot-state API at `http://127.0.0.1:8765/state` so the FR3 control / LSL tooling can distinguish kinesthetic sub-modes.

Current behavior:
- `kt_active` is handled by the outer FR3 control launcher when this GUI is started or stopped
- `teaching_active = 1` while actual teach / recording is active
- `teaching_active = 0` when teach stops or is cleaned up
- `running_active = 1` while trajectory playback is active
- `running_active = 0` when playback finishes, aborts, or is cleaned up
- gravity-compensation mode by itself does **not** mark `teaching_active` as on

This separation is intentional so downstream LSL consumers can distinguish:
- kinesthetic GUI open
- actual teaching / recording
- trajectory playback

## Main application

The main application is [`franka_teach_run_gui_v2.py`](/home/parc/franka_kinesthetic_teaching_GUI/franka_teach_run_gui_v2.py). It is launched by [`run_gui.sh`](/home/parc/franka_kinesthetic_teaching_GUI/run_gui.sh), which sources ROS and then runs the Tkinter app.

The GUI exposes these actions:
- `Start Teach (Record)`
- `Start Gravity Mode`
- `Open Gripper`
- `Close Gripper`
- `Run Trajectory`
- `Pause`
- `Resume`
- `Kill`

## Repo contents

- `franka_teach_run_gui_v2.py`: Tkinter GUI for teach, gravity, gripper, and playback workflows
- `run_gui.sh`: convenience launcher for the GUI
- `franka_teach_minimal.launch.py`: reduced teach/gravity launch used by the GUI
- `franka_teach.config.yaml`: robot config passed into the minimal teach launch
- `record_joint_trajectory.py`: records joint states to CSV
- `playback_joint_trajectory.py`: loads a CSV, applies bounded smoothing, blends from the current pose, executes continuous arm trajectories, supports pause/resume, auto-pauses on safety signals, and replays recorded gripper events

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
- `soft_stop_gravity_controller` on the `Humble_KT_failsafe` branch
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

It then waits for the playback stack to become ready before running [`playback_joint_trajectory.py`](/home/parc/franka_kinesthetic_teaching_GUI/playback_joint_trajectory.py) on the selected CSV.

## Features

### Teach mode

- `Start Teach (Record)` is the full recording workflow, not just a gravity-compensation shortcut
- If the reduced teach stack is not already running, `Start Teach (Record)` arms the recorder first and only then enables gravity compensation
- This avoids the old startup window where the arm could already be movable before the recorder had latched a valid Franka joint-state topic
- If `Start Gravity Mode` is already active, `Start Teach (Record)` reuses that running stack and starts only the recorder
- Records joint motion while the arm is moved by hand
- The GUI waits for the recorder to latch a usable joint-state topic before it marks teaching as active
- Publishes `teaching_active = 1` while recording is active and clears it when recording stops
- `Start Teach (Record)` prompts for an optional CSV filename before recording starts
- `Start Teach (Record)` starts `record_joint_trajectory.py` and stores the target CSV path
- Uses a timestamped filename if you leave the prompt blank
- Records `Open Gripper` and `Close Gripper` button presses as timestamped `open` and `close` events
- `Stop Teach (Save)` merges confirmed gripper events into the CSV in timestamp order when teach mode stops
- `Stop Teach (Save)` stops the recorder and the teach stack, then resets the GUI back to its idle teach button state
- Uses the already-running gripper action server from the teach/gravity stack
- Avoids a separate gripper connection while arm control is active

### Gravity mode

- `Start Gravity Mode` is a separate gravity-compensation workflow, not the same thing as `Start Teach (Record)`
- `Start Gravity Mode` starts the same reduced ROS stack without prompting for a CSV filename and without starting `record_joint_trajectory.py`
- Lets you put the arm into gravity compensation independently from recording
- On the `Humble_KT_failsafe` branch, gravity mode uses an experimental soft-stop controller from `~/franka_ws_jointfailsafe`
- In that experimental mode, free-space motion is intended to stay close to normal gravity compensation, while approaching FR3 joint limits should feel like a soft physical boundary
- Exposes the gripper action server from the same bringup stack
- `Start Gravity Mode` changes to `Stop Gravity Mode` while the stack is active
- `Start Gravity Mode` and `Stop Gravity Mode` do not assert `teaching_active` by themselves; gravity mode is kept distinct from actual teach / recording
- `Stop Gravity Mode` shuts down the gravity stack and resets the button without saving a recording unless you separately started `Start Teach (Record)`

### GUI gripper buttons

- `Open Gripper` sends a Franka `Move` action
- `Close Gripper` sends a Franka `Grasp` action
- During teach/gravity mode, the GUI uses the gripper action server from the already-running teach stack
- Outside teach/gravity mode, the GUI starts a standalone `franka_gripper` launch, sends the command, then shuts it down again
- The GUI refuses to start a second gripper node while teach/gravity arm control is active but no gripper action server is available

### Run mode

- `Run Trajectory` lets you choose a saved CSV from the GUI
- `Run Trajectory` is disabled unless the robot is in a clean run-ready state
- Run readiness requires no active teach mode, no active gravity mode, no teach/gravity shutdown still in progress, no active playback, and no in-flight standalone gripper command
- `Run Trajectory` launches the FR3 MoveIt/controller stack
- Publishes `running_active = 1` while trajectory playback is active and clears it when playback finishes or aborts
- `Run Trajectory` first enters a `Preparing…` phase while the MoveIt/controller stack starts
- `Run Trajectory` waits for a trajectory-controller subscriber and a gripper action server before starting playback
- The playback node waits for joint state feedback and a trajectory-controller subscriber before publishing
- Smooths recorded waypoints during playback
- Applies a small replay-only inward joint-limit margin before sending points to `fr3_arm_controller`
- Blends from the robot's current joint pose into the recorded trajectory start when needed
- Executes continuous arm trajectories through `follow_joint_trajectory` during normal playback
- Supports `Pause` and `Resume` during playback
- On pause, cancels the active arm trajectory goal and holds the current pose
- On resume, rebuilds one new continuous trajectory from the current pose into the remaining recorded path
- Monitors Franka robot-state safety signals during playback
- Auto-pauses on Cartesian contact, collision indicators, or external joint torque above the software threshold
- Uses a conservative software external joint torque auto-pause threshold of `5.0 Nm`
- Uses Franka's hardware collision/reflex layer as the final safety backstop if contact escalates faster than software cancellation can settle the arm
- Holds the arm only when recorded gripper events must execute
- Waits briefly for the preferred Franka joint-state topic before starting from a fallback topic
- On `Humble_KT_failsafe`, playback still runs through the normal MoveIt / `fr3_arm_controller` stack even though teach / gravity uses the experimental soft-stop controller

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

On the `Humble_KT_failsafe` branch, `ROS_SETUP` is intentionally pointed at:

- `~/franka_ws_jointfailsafe/install/setup.bash`

Teach/gravity robot settings live in [`franka_teach.config.yaml`](/home/parc/franka_kinesthetic_teaching_GUI/franka_teach.config.yaml).

Playback constants such as smoothing, blend timing, and gripper action candidates live in [`playback_joint_trajectory.py`](/home/parc/franka_kinesthetic_teaching_GUI/playback_joint_trajectory.py).
That file also contains the software auto-pause thresholds and safety-monitor settings used during playback.
Recorder topic-selection safeguards live in [`record_joint_trajectory.py`](/home/parc/franka_kinesthetic_teaching_GUI/record_joint_trajectory.py).
GUI session logs are written under [`logs/`](/home/parc/franka_kinesthetic_teaching_GUI/logs:1).

Important branch note:
- on `Humble_KT_failsafe`, teach / gravity uses the experimental backend in `~/franka_ws_jointfailsafe`
- trajectory playback also launches from that workspace, but still uses the normal MoveIt / trajectory-controller path rather than the soft-stop gravity controller

## Running

```bash
cd ~/franka_kinesthetic_teaching_GUI
./run_gui.sh
```

## Usage

### GUI User Guide

The main window is intentionally simple: two rows of control buttons, a one-line status field, a scrollable runtime log, and an `Open CSV Dir…` shortcut for recorded files.

### Top-row controls

#### `Start Teach (Record)` / `Stop Teach (Save)`

- `Start Teach (Record)` begins the teaching workflow.
- It first prompts for an optional CSV filename. If you leave it blank, the GUI creates a timestamped filename.
- If gravity compensation is not already active, the GUI starts the recorder first and only then starts the reduced teach stack so the arm can be moved by hand.
- If gravity compensation is already active, the GUI keeps that stack running and starts only the recorder.
- The GUI waits for `record_joint_trajectory.py` to latch a usable Franka joint-state topic before it sets `teaching_active = 1`, changes the button label to `Stop Teach (Save)`, and shows the active recording status.
- The operator should not move the robot until the GUI indicates that recording is active.
- While teaching is active, `Run Trajectory`, `Start Gravity Mode`, and `Open CSV Dir…` are disabled so the recording session is not interrupted.
- Any `Open Gripper` and `Close Gripper` presses during teaching are timestamped so they can be added to the same recording.
- When you click `Stop Teach (Save)`, the GUI stops the recorder and teach processes, appends any recorded gripper events to the CSV, clears `teaching_active`, and changes the button back to `Start Teach (Record)`.

#### `Run Trajectory`

- Opens a file picker so you can choose a recorded CSV.
- The button is disabled unless the robot is actually ready to run.
- In practice, teach mode, gravity mode, leftover teach/gravity shutdown, prior playback processes, and temporary standalone gripper activity all block the button.
- Starts the FR3 MoveIt/controller launch used for playback.
- Changes its label to `Preparing…` while the playback stack is being brought up.
- Waits for a trajectory-controller subscriber and a gripper action server before starting `playback_joint_trajectory.py`.
- If those readiness checks fail, playback is aborted cleanly and the status line shows the readiness failure instead of attempting a bad run.
- Changes its label to `Running…` while playback is in progress.
- Re-enables the rest of the GUI when playback finishes or aborts.

#### `Pause` / `Resume`

- `Pause` is available only while trajectory playback is running.
- `Pause` cancels the active continuous arm trajectory goal and commands the arm to hold its current pose.
- `Resume` is available only after a successful pause.
- `Resume` rebuilds the remaining path from the robot's current measured pose and continues from the remaining recorded points instead of restarting the full trajectory.
- Normal playback is continuous; pause/resume interrupts and reissues continuous execution only when requested.
- The GUI also enters the same paused state automatically when the playback node requests a safety auto-pause.

#### `Kill`

- Immediately sends termination signals to all teach, playback, and temporary gripper processes started by the GUI.
- Clears GUI mode flags, resets button labels, and sets the status line to `Active processes killed.`.
- This is the emergency stop/cleanup button for the application layer. It is not a substitute for the robot's physical safety stop.

### Second-row controls

#### `Start Gravity Mode`

- `Start Gravity Mode` enables gravity compensation so the arm can be moved freely by hand without starting a recording.
- It starts the reduced gravity-compensation ROS launch and changes the button to `Stop Gravity Mode`.
- It does not prompt for a filename and does not start `record_joint_trajectory.py`.
- When you click `Stop Gravity Mode`, the GUI shuts that stack down and returns the button to `Start Gravity Mode`.

#### `Open Gripper`

- Sends a Franka `Move` action to open the gripper to the configured open width.
- If teaching is active, also records an `open` gripper event with a timestamp so it can be merged into the CSV when teaching stops.
- During teach/gravity mode, uses the gripper action server from the already-running teach stack.
- Outside teach/gravity mode, temporarily launches a standalone gripper node, sends the command, then shuts that node down.

#### `Close Gripper`

- Sends a Franka `Grasp` action to close the gripper using the configured width, speed, force, and epsilon values.
- If teaching is active, also records a `close` gripper event for later CSV merge.
- Uses the same action-server logic as `Open Gripper`: shared server during teach/gravity mode, temporary standalone node otherwise.

### Footer and feedback area

#### Status line

- Shows the current high-level GUI state such as `Idle`, `Gravity compensation active.`, `Recording… Move arm by hand to teach.`, `Running trajectory…`, or the most recent failure message.

#### Runtime log

- Streams launch output, recorder/playback progress, gripper command results, and teach/gravity failure messages into the text log.
- This is the first place to look if a launch fails, a gripper server is missing, or playback shuts down unexpectedly.
- Each GUI launch also writes the same stream to a timestamped session log file under [`logs/`](/home/parc/franka_kinesthetic_teaching_GUI/logs:1).
- Uncaught GUI callback exceptions and background-thread exceptions are written to that session log as well.

#### `Open CSV Dir…`

- `Open CSV Dir…` opens the GUI's current working directory in the system file browser with `xdg-open`.
- This is the directory where recordings are saved by default when you do not provide a different filename or path.

### Record a trajectory

1. Optionally click `Start Gravity Mode` if you want gravity compensation without recording yet.
2. Click `Start Teach (Record)`.
3. Optionally enter a custom CSV filename, or leave it blank for a timestamped default.
4. If the teach stack is not already running, let the GUI arm the recorder before gravity compensation becomes active.
5. Wait for the recorder to start receiving valid Franka joint states and for the GUI to indicate that recording is active.
6. Only then move the arm by hand to demonstrate the motion.
7. Optionally use `Open Gripper` or `Close Gripper` during teaching if you want those actions recorded.
8. Click `Stop Teach (Save)`.

### Play back a trajectory

1. Click `Run Trajectory`.
2. Choose a previously recorded CSV.
3. The GUI only allows this if the robot is already in a clean run-ready state.
4. The GUI launches the FR3 MoveIt/controller stack and enters `Preparing…`.
5. The GUI waits for a trajectory-controller subscriber and a gripper action server.
6. The playback node waits for valid joint states and an active trajectory-controller subscriber.
7. Playback executes a continuous arm trajectory that blends from the current pose into the recording when needed.
8. If you click `Pause`, the active arm trajectory is canceled and the arm is held at its current pose.
9. If you click `Resume`, playback rebuilds one new continuous trajectory from the current pose into the remaining recorded path.
10. If the CSV contains `gripper` rows, playback holds the arm only when those recorded `open` and `close` events must execute.

Recent validated behavior:
- the recorder-first teach startup reduced a previously bad start mismatch from about `0.933 rad` to about `0.335 rad` in the next recorded run
- that later run completed successfully, with only software torque auto-pauses and clean resumes

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

### Pause and resume behavior

- Normal arm playback is issued as a continuous `FollowJointTrajectory` action goal
- `Pause` cancels the active trajectory goal instead of chunking normal playback into short segments
- After cancelation, the player holds the current pose
- `Resume` estimates already-consumed recorded points, then rebuilds a new continuous goal for the remaining path
- Re-blending happens on startup and after an actual resume, not continuously during normal playback

### Safety auto-pause behavior

- During playback, the node subscribes to Franka robot state from `franka_robot_state_broadcaster`
- Playback auto-pauses if any of these early-warning conditions become active
- Cartesian contact indicator
- Collision indicator
- External joint torque estimate above the software threshold
- The current software external joint torque auto-pause threshold is `5.0 Nm`
- When auto-pause triggers, the active `FollowJointTrajectory` goal is canceled and the arm is commanded to hold its current pose
- The GUI exposes this as a paused state so the operator can adjust the tool or environment and press `Resume`
- `Resume` rebuilds a new continuous trajectory from the current measured pose into the remaining recorded path
- This software layer is intentionally earlier and more conservative than Franka's hardware reflex layer
- It does not replace Franka collision/reflex safety; a fast or hard contact can still escalate to a hardware reflex before software cancellation fully settles the arm

### Franka contact and reflex relationship

- Franka's collision behavior distinguishes `contact` from `collision`
- Contact corresponds to the lower threshold band and is exposed through the robot-state indicators
- Collision corresponds to the upper threshold band and can stop the robot in hardware
- In this repo, the software auto-pause reacts to contact/collision indicators and low external torque before the motion should reach a full reflex stop
- If the log reports `cartesian_reflex` or `joint_reflex`, that means Franka's built-in hardware safety stopped the motion anyway

### Replay safety margin

- On playback only, points that get too close to the FR3 joint limits are clamped slightly inward before publishing
- This inward clamp is controlled by `REPLAY_LIMIT_MARGIN_RAD = 0.06`
- The original CSV on disk is not modified; the adjustment only affects the in-memory replayed trajectory
- If any points are adjusted, the playback log reports which joints were affected and how many points were changed

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
- The GUI still depends on ROS 2 / DDS / controller startup health; if the playback controller fails to load, the readiness gate will keep `Run Trajectory` from starting instead of forcing playback anyway
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

## License

This repository is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

This branch targets Ubuntu 22.04 with ROS 2 Humble and integrates the failsafe backend workspace (~/franka_ws_jointfailsafe). This project interfaces with ROS 2, the Franka ROS 2 stack, MoveIt, and related system packages, which are governed by their own licenses. Review those licenses before redistributing a bundled application, Docker image, or robot-control deployment environment.


