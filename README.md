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
- `tt_robot_new`: Main robot package with 5-DOF table tennis robot
- `tt_robot_new_moveit_config`: MoveIt configuration for the robot
- `tt_robot_control`: Package containing robot movement and simulation programs

## Key Features

- **Ball Position Tracking**: Computer vision-based tracking of table tennis balls
- **Rally Simulator**: Simulation of table tennis rallies with randomized ball positions
- **5-DOF Robot Control**: Control of a table tennis robot with 5 degrees of freedom:
  - NEMA 23 powered prismatic joint for linear motion
  - NEMA 23 powered rotational joint
  - ST3215 servo-controlled main arm, sub arm, and wrist joints
- **MoveIt Integration**: Path planning and execution using MoveIt

## Coming Soon
- **Advanced Ball Prediction**: Implementation of Extended Kalman Filter (EKF) for accurate ball trajectory prediction
- **Automatic Robot Movement**: Automated robot control to intercept predicted ball positions

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

### Running the Real System

1. Launch the MoveIt demo:
```bash
ros2 launch tt_robot_new_moveit_config demo.launch.py
```

2. In a new terminal, launch the RealSense camera:
```bash
ros2 launch realsense2_camera rs_launch.py
```

3. In a new terminal, run the ball tracking node:
```bash
ros2 run ball_tracker ball_position_tracker
```

4. In a new terminal, run the ball position predictor (coming soon):
```bash
# This will be available in a future update
# ros2 run ball_tracker ball_predictor
```

5. In a new terminal, run the robot control program (coming soon):
```bash
# This will be available in a future update
# ros2 run tt_robot_control move_program
```

### Running the Rally Simulator

To test the robot in simulation with randomized ball positions:

```bash
ros2 run tt_robot_control rally_simulator [number_of_serves]
```

The rally simulator:
- Generates random ball positions on the robot's side (x: 0-0.2m, y: -0.8-0.8m, z: 0-0.5m)
- Plans and executes robot movements to intercept the ball
- Performs realistic robot swing motions (forehand and backhand)
- Uses optimized motor speeds for NEMA 23 and ST3215 actuators

## Future Ball Prediction and Filtering

The system will soon implement an Extended Kalman Filter (EKF) for ball trajectory prediction with:
- Physics-based motion model accounting for gravity and bounces
- Adaptive measurement update to handle noisy measurements
- Future trajectory prediction to prepare the robot before the ball arrives
- Visualization of the predicted trajectory for debugging

## Configuration

- The YOLO model weights (`best.pt`) should be downloaded separately and placed in the workspace root
- Camera configuration can be modified in the RealSense launch files
- Robot configuration parameters are in the MoveIt config package
- Rally simulator parameters can be modified in `rally_simulator.cpp`

## Troubleshooting

If you encounter any issues:
1. Ensure all dependencies are properly installed
2. Check if the RealSense camera is properly connected
3. Verify CUDA is properly set up for YOLO inference
4. Make sure all ROS2 environment variables are properly set
5. Verify that the YOLO weights file (`best.pt`) is properly downloaded and placed in the correct location
6. Ensure you've sourced the workspace in each new terminal
7. Check the ROS2 topics to verify data is being published correctly:
   ```bash
   ros2 topic list
   ros2 topic echo /ball_position
   ```

## Contributing

Feel free to submit issues and pull requests.

## License

[Your License Here] 

# Table Tennis Robot Control Package

This package contains the control software for a table tennis robot that uses NEMA motors and ST3215 servos.

## Dependencies

- ROS2 Humble
- Python3
- MoveIt2
- Intel RealSense SDK
- Python3-dev

## Installation

1. Clone the repository with submodules:
```bash
git clone --recursive git@github.com:YOUR_USERNAME/tt_package.git
cd tt_package
```

2. If you haven't cloned with submodules, initialize them:
```bash
git submodule update --init --recursive
```

3. Build the package:
```bash
colcon build --packages-select tt_robot_control
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Hardware Setup

1. Connect the NEMA motors to `/dev/ttyUSB0`
2. Connect the ST3215 servos to `/dev/ttyUSB1`
3. Connect the RealSense camera to a USB 3.0 port

## Usage

1. Start the robot controller:
```bash
ros2 run tt_robot_control tt_robot_controller
```

2. The controller will:
   - Subscribe to joint states from MoveIt
   - Control NEMA motors for joints 1 & 2
   - Control ST3215 servos for joints 3, 4, & 5
   - Convert joint positions to appropriate motor commands

## Joint Configuration

- Joint 1 (NEMA): Linear motion along table width
- Joint 2 (NEMA): Rotational motion for arm orientation
- Joint 3 (ST3215): Shoulder joint
- Joint 4 (ST3215): Elbow joint
- Joint 5 (ST3215): Wrist joint

## Troubleshooting

1. If you get permission errors for USB devices:
```bash
sudo usermod -a -G dialout $USER
```
Then log out and log back in.

2. If the motors don't respond:
   - Check USB connections
   - Verify device paths in the code
   - Check motor power supply

## License

[Your chosen license] 