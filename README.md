# TT Package - ROS2 Table Tennis Robot System

This repository contains a ROS2 package for a table tennis robot system that includes ball tracking, computer vision using YOLO, RealSense camera integration, and robot control with MoveIt.

## Prerequisites

- Ubuntu 22.04 (or compatible Linux distribution)
- ROS2 Humble
- Python 3.8+
- Intel RealSense SDK 2.0
- CUDA (for YOLO inference)

## Package Components

The workspace contains the following packages:
- `ball_tracker`: Package for tracking table tennis balls
- `yolo_ros`: ROS2 integration for YOLO object detection
- `realsense-ros`: RealSense camera drivers and ROS2 integration
- `tt_robot_new`: Main robot package
- `tt_robot_new_moveit_config`: MoveIt configuration for the robot
- `move_program`: Package containing robot movement programs

## Installation

1. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/Chinmay-Prashanth/tt_package.git
cd ..
```

3. Install dependencies:
```bash
sudo apt update
sudo apt install ros-humble-realsense2-camera ros-humble-moveit
rosdep install --from-paths src --ignore-src -r -y
```

4. Download the YOLO model weights:
   - Download the `best.pt` file from [your_download_link]
   - Place it in the root directory of the workspace

5. Build the workspace:
```bash
colcon build
source install/setup.bash
```

## Usage

For each terminal, first source the workspace:
```bash
source install/setup.bash
```

Then follow these steps in order:

1. Launch the MoveIt demo:
```bash
ros2 launch tt_robot_new_moveit_config demo.launch.py
```

2. In a new terminal, run the random movement program:
```bash
ros2 run move_program move_random
```

3. In a new terminal, run the joint states converter:
```bash
ros2 run move_program joint_state_converter
```

4. In a new terminal, launch the RealSense camera:
```bash
ros2 launch realsense2_camera rs_launch.py
```

5. In a new terminal, run the ball tracking node:
```bash
ros2 run ball_tracker ping_pong_detector
```

6. In a new terminal, run the ball position prediction node (to be implemented):
```bash
ros2 run move_program ball_predictor
```

7. Finally, run the hitting motion node (to be implemented):
```bash
ros2 run move_program hitting_motion
```

## Configuration

- The YOLO model weights (`best.pt`) should be downloaded separately and placed in the workspace root
- Camera configuration can be modified in the RealSense launch files
- Robot configuration parameters are in the MoveIt config package

## Troubleshooting

If you encounter any issues:
1. Ensure all dependencies are properly installed
2. Check if the RealSense camera is properly connected
3. Verify CUDA is properly set up for YOLO inference
4. Make sure all ROS2 environment variables are properly set
5. Verify that the YOLO weights file (`best.pt`) is properly downloaded and placed in the correct location
6. Ensure you've sourced the workspace in each new terminal

## Contributing

Feel free to submit issues and pull requests.

## License

[Your License Here] 