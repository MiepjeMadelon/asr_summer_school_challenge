# ASR Summer School: Search-and-Rescue Challenge

## Activity overview

This repository supports a **search-and-rescue robotics laboratory** for master's students and early PhD students. The activity is carried out in teams using a TurtleBot3 Burger.

The robot is equipped with:

- a 2D LiDAR for mapping and obstacle avoidance;
- an RGB-D camera for AprilTag detection;
- wheel odometry and an IMU for motion estimation;
- ROS 2 software, including `slam_toolbox`, `nav2`, and `apriltag_ros`.

Each team starts in an unknown indoor environment and develops an autonomous system capable of:

1. building a 2D occupancy map;
2. exploring the environment autonomously;
3. detecting as many AprilTags as possible;
4. associating every detection with its unique tag ID;
5. estimating and storing each tag position in the map frame;
6. returning to the starting position before the available time expires;
7. saving both the occupancy map and a semantic map of the detected targets.

The final task is a fixed-time challenge: **find the largest possible number of AprilTags and return home**. A successful solution must therefore balance exploration, perception, navigation, and the time required for a safe return.

## Repository contents

The ROS 2 workspace contains the following main packages:

```text
ws/src/
├── asr_summer_school/       # Laboratory bringup, launch, and configuration
├── laser_filters/           # LiDAR filtering package
├── turtlebot3_perception/   # RGB-D camera, AprilTag, and landmark support
└── README.md
```

### `asr_summer_school`

`asr_summer_school` is an `ament_cmake` ROS 2 package that collects the launch files and parameter sets used to operate the robot during the laboratory. Its `CMakeLists.txt` installs the `launch/` and `config/` directories into the package share directory.

```text
asr_summer_school/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── param_nav2.yaml
│   ├── param_slam_toolbox.yaml
│   └── param_teleop.yaml
├── include/asr_summer_school/
├── launch/
│   ├── bringup.launch.py
│   ├── nav2.launch.py
│   ├── slam_toolbox.launch.py
│   └── teleop.launch.py
└── src/
```

The package currently contains configuration and orchestration files; the `include/` and `src/` directories are available for code developed during the laboratory.

#### Launch files

- `bringup.launch.py` starts the TurtleBot3 base, SLAM, joystick teleoperation,  RGB-D camera, and AprilTag detector as one integrated system.
- `slam_toolbox.launch.py` starts asynchronous `slam_toolbox`, manages its lifecycle, and inserts a `laser_filters` scan-to-scan filter before SLAM.
- `nav2.launch.py` starts the Nav2 navigation stack. It supports mapping or localization, namespaces, composition, respawning, simulation time, and a custom parameter file.
- `teleop.launch.py` starts `joy_linux` and `teleop_twist_joy`, allowing the TurtleBot3 to be driven with a game controller.

#### Configuration files

- `param_slam_toolbox.yaml` configures the LiDAR binning filter and `slam_toolbox`, including frames, filtered scan topic, map resolution, scan matching, and loop closure.
- `param_nav2.yaml` configures localization, behavior-tree navigation, controller and planner servers, costmaps, obstacle processing, recovery behaviors, and velocity limits for the TurtleBot3.
- `param_teleop.yaml` defines the joystick axes, enable button, and linear and angular velocity scales.

### Supporting packages

- `laser_filters` provides the filter chain used to preprocess LiDAR scans. In the supplied SLAM configuration, scans are binned and published on `/scan_filtered` before being consumed by `slam_toolbox`.
- `turtlebot3_perception` provides the camera and AprilTag launch support used by the main bringup file. It also contains `landmark_msgs`, which defines messages for representing detected landmarks.

The current package supplies the robot bringup, mapping, navigation, teleoperation, and perception foundations. The autonomous exploration policy, unique-tag management, transformation and storage of detections in the map frame, semantic-map export, and timed return-to-start behavior are the main components to be developed as part of the challenge.
