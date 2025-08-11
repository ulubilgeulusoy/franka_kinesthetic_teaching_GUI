# franka_kinesthetic_teaching_GUI

A Tkinter-based GUI for **kinesthetic teaching** and **trajectory playback**  
on the Franka Research 3 robotic arm using ROS 2.

## Features
- **Teach mode**:  
  - Puts FR3 into gravity compensation mode  
  - Records a joint trajectory while you move the arm by hand
- **Run mode**:  
  - Select a saved CSV trajectory and execute it via MoveIt
- **Thread-safe** logging to display ROS output inside the GUI

## Requirements
- ROS 2 Humble (tested)
- `franka_ros2` packages installed and configured
- Python 3 with Tkinter (`sudo apt install python3-tk`)

## Installation
```bash
git clone https://github.com/YOUR_USERNAME/franka_kinesthetic_teaching_GUI.git
```


## Running
# Make sure that robot arm is online and joints unlocked
# FCI mode is active
# Open new terminal
cd franka_kinesthetic_teaching_GUI
chmod +x franka_teach_run_gui_v2.py
./franka_teach_run_gui_v2.py

## Usage
# Click Start Teach (Record) to put the robot in gravity compensation mode.
# Move the arm by hand to teach a trajectory.
# Click Stop Teach (Save) to store it as a CSV.
# Click Run Trajectory, select a CSV, and the robot will execute it.

## Acknowledgements
# This project was developed with assistance from ChatGPT 5.0 by OpenAI.
