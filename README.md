# Project developed for the Topic Highlight Course (University of Bologna) - Active Sensing and Perceptions

This project utilizes a depth camera mounted on a TurtleBot3 robot to detect and track a person's position within a home environment. The system plans navigation paths to follow the detected person while avoiding obstacles, leveraging existing ROS2 packages.

Project Team:
Enrico Battistini
Filippo Ugolini (filippougolini19@gmail.com / filippo.ugolini@studio.unibo.it) | GitHub: https://github.com/Ugo-Filo01
Jacopo Subini

# TurtleBot3 Workspace — Localization, Detection, Following, and Simulation

This repository contains a ROS 2 workspace integrating three custom Python packages and the official TurtleBot3 simulation stack:

- `autonomous_localization`: initializes and assists robot localization using AMCL, publishing an initial pose and rotating until the covariance falls below a threshold.
- `detection_pkg`: detects a person in RGB-D images, estimates their position in the `map` frame, and publishes a navigation goal.
- `follow_person_pkg`: subscribes to the goal and drives the robot using the Nav2 action server to follow at a safe distance.
- `turtlebot3_simulations`: TurtleBot3 Gazebo assets and launch files (`turtlebot3_gazebo`, `turtlebot3_fake_node`).

## Repository Layout

```
src/
  autonomous_localization/
  detection_pkg/
  follow_person_pkg/
  turtlebot3_simulations/
```

- Package manifests and Python entry points are defined in each package’s `setup.py`.
- TurtleBot3 simulation packages include Gazebo worlds, models, and launch files.

## Prerequisites

- ROS 2 (Humble or newer) with Nav2 installed
- Gazebo Classic for simulation (TurtleBot3 Gazebo)
- TurtleBot3 messages and dependencies
- Python 3.10+
- Python packages: `opencv-python`, `numpy`, `scipy`, `pykdl`, `cv_bridge` (ROS 2), `rclpy`

> Windows users: ROS 2 runs on Windows; Gazebo support may be limited. Consider using WSL2/Ubuntu for simulation. On Windows, use `. install\setup.bat`; on Linux/macOS, use `source install/setup.bash`.

## Setup

1. Install ROS 2 and Nav2 following the official docs.
2. Clone this repository into your ROS 2 workspace.
3. Build with `colcon`:

```bash
# From the repository root
colcon build
# Windows (PowerShell)
. install\setup.bat
# Linux/macOS
source install/setup.bash
```

## Simulation (TurtleBot3)

Start a minimal Gazebo world and RViz:

```bash
# Empty world
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Fake node (RViz demo)
ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
```

## Packages & Usage

### autonomous_localization

- Node: `autonomous_localization`
- Entry point: `autonomous_localization = autonomous_localization.autonomous_localization:main`
- Subscriptions: `amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`), `odom` (`nav_msgs/Odometry`)
- Publications: `initialpose` (`geometry_msgs/PoseWithCovarianceStamped`), `cmd_vel` (`geometry_msgs/Twist`)
- Behavior: Publishes an initial pose, rotates (`cmd_vel`) and checks AMCL covariance until it falls below a threshold.

Run:

```bash
ros2 run autonomous_localization autonomous_localization
```

### detection_pkg

- Node: `detection_node`
- Entry point: `detection_node = detection_pkg.detection_node:main`
- Subscriptions: `/camera/image_raw` (RGB), `/camera/depth/image_raw` (Depth), `/camera/depth/camera_info`
- Publications: `goal_pose` (`geometry_msgs/PoseStamped`), `/image_recording` (`sensor_msgs/Image`)
- TF frames used: `map`, `base_link`, `camera_depth_optical_frame`
- Model files expected in the package folder:
  - `frozen_inference_graph.pb`
  - `mask_rcnn_inception_v2_coco_2018_01_28.pbtxt`
  - `mscoco_labels.names`

Important: The paths to the `.pb` and `.pbtxt` are hardcoded. Update them to point to the packaged files under `src/detection_pkg/detection_pkg`. Edit the variables `textGraph` and `modelWeights` in [src/detection_pkg/detection_pkg/detection_node.py](src/detection_pkg/detection_pkg/detection_node.py), or set them via a parameterization if you refactor.

Run:

```bash
ros2 run detection_pkg detection_node
```

### follow_person_pkg

- Node: `follow_person_node`
- Entry point: `follow_person_node = follow_person_pkg.follow_person_node:main`
- Subscriptions: `/goal_pose` (`geometry_msgs/PoseStamped`)
- Uses Nav2 action server: `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
- Behavior: Computes an offset to maintain a safe following distance and sends Nav2 goals. Requires Nav2 to be running.

Run:

```bash
# Ensure Nav2 is active (e.g., bringup)
ros2 run follow_person_pkg follow_person_node
```

## Typical Workflow

1. Start TurtleBot3 simulation:
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```
2. Start AMCL/Localization stack (e.g., Nav2 bringup with map server).
3. Run `autonomous_localization` to initialize pose and converge covariance:
   ```bash
   ros2 run autonomous_localization autonomous_localization
   ```
4. Start person detection to publish goals:
   ```bash
   ros2 run detection_pkg detection_node
   ```
5. Start follow-person controller (Nav2 must be available):
   ```bash
   ros2 run follow_person_pkg follow_person_node
   ```

## Development & Testing

- Each package includes basic tests (`test/`) for linting (`flake8`, `pep257`) and copyright checks.
- Run tests after building:

```bash
colcon test
colcon test-result --verbose
```

## Notes & Tips

- Camera topics: ensure your simulated/real camera publishes `/camera/image_raw`, `/camera/depth/image_raw`, and `/camera/depth/camera_info`.
- TF availability: the detection node expects `map → camera_depth_optical_frame` and `map → base_link`. Verify TF tree in RViz/`tf2_tools`.
- Paths in `detection_pkg`: avoid absolute paths; keep models beside the node and reference them with package-relative paths.
- Safe distances and thresholds can be tuned inside each node source.

## License

- `turtlebot3_simulations` contains upstream license files. The root project license can be added here when chosen.

## Acknowledgements

- TurtleBot3 by ROBOTIS
- ROS 2 community and Nav2 contributors
