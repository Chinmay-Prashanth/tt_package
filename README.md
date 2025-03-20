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

- **Extended Kalman Filter (EKF)**: Advanced filtering for accurate ball trajectory prediction (in development)
- **Predictive Robot Movement**: Robot movement to predicted ball positions (in development)

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

### Running the Current System

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

4. In a new terminal, run the robot control program (basic movements):
```bash
ros2 run tt_robot_control move_random
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

## Future Ball Prediction and Movement Features (Coming Soon)

The system will soon include an Extended Kalman Filter (EKF) for ball trajectory prediction with:
- Physics-based motion model accounting for gravity and bounces
- Adaptive measurement update to handle noisy measurements
- Future trajectory prediction to prepare the robot before the ball arrives
- Visualization of the predicted trajectory for debugging

Additionally, a specialized move_program will be implemented to:
- Process predicted ball positions in real-time
- Calculate optimal interception points
- Plan and execute efficient robot movements to meet the ball
- Adapt to changing predictions with path replanning

## Development Roadmap

1. ✅ Ball position tracking
2. ✅ Basic robot movements
3. ✅ Rally simulator
4. 🔄 Ball trajectory prediction (in progress)
5. 🔄 Predictive robot movement (in progress)
6. ⬜ Full game capability

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