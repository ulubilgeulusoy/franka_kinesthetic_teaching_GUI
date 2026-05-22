# THIRD_PARTY_NOTICES

This project uses or interfaces with third-party software components that are distributed separately and remain under their respective licenses.

## Runtime and Framework Components
- Python / Tkinter (Python Software Foundation License)
- ROS 2 / rclpy (Apache License 2.0)

## Robotics and Control Stack (External)
- franka_ros2, including franka_msgs and franka_bringup (primarily Apache License 2.0)
- franka_fr3_moveit_config (typically Apache License 2.0 within franka_ros2 ecosystem)
- MoveIt and ros2_control components (primarily Apache License 2.0)
- libfranka (Apache License 2.0)
This repository does not vend or relicense those upstream packages; it invokes them through an installed ROS environment/workspace. You are responsible for reviewing and complying with upstream licenses when redistributing a bundled system image or deployment environment.

