# Wall-E ROS 2 Workspace

A ROS 2 workspace for simulating a differential-drive Wall-E robot with Gazebo and RViz. The project includes a robot description (URDF/Xacro), STL meshes, Gazebo worlds, launch files, and a ROS↔Gazebo bridge configuration. Future work will add control and AI integration.

## At a glance

- ROS 2 package: `wall-e`
- Simulation: Gazebo (Gazebo Sim / Ignition) via `ros_gz_bridge`
- Visualization: RViz with preconfigured display file
- Robot description: URDF and Xacro with STL meshes
- Worlds: example SDF worlds for empty and diff-drive setups
- Launch files: `gazebo.launch.py`, `rviz.launch.py`

## Repository layout

Workspace root (this folder):

- `src/wall-e/`
	- `CMakeLists.txt`, `package.xml` — ROS 2 package metadata and build rules
	- `launch/`
		- `gazebo.launch.py` — starts Gazebo sim and ROS–Gazebo bridge
		- `rviz.launch.py` — starts RViz with a provided config
	- `urdf/`
		- `robot.urdf`, `robot.urdf.xacro` — robot description
	- `meshes/` — STL meshes for the robot links (`base_link.STL`, `lf_link.STL`, `lb_link.STL`, `rf_link.STL`, `rb_link.STL`)
	- `worlds/` — Gazebo worlds (`empty.sdf`, `diff_drive.sdf`)
	- `rviz/display_config.rviz` — RViz display configuration
	- `config/ros_gz_bridge_config.yaml` — ROS↔Gazebo bridge topics config

Build artifacts are under `build/`, installed files under `install/` (created by colcon).

## Requirements

- ROS 2 (tested with modern distros like Humble/Irons/rolling; adjust as needed)
- Gazebo Sim (Ignition) and `ros_gz_bridge`
- colcon and ament build tools

On Ubuntu, you can install ROS 2 and Gazebo per the official docs; ensure `ros-gz-bridge` (or distro-specific packages) is installed.

## Build

```bash
# From the workspace root
colcon build --symlink-install

# Source the overlay for the current shell
source install/setup.bash
```

## Run the simulation

Start Gazebo with the robot and bridge:

```bash
ros2 launch wall-e gazebo.launch.py
```

Start RViz in another terminal (after sourcing the workspace):

```bash
ros2 launch wall-e rviz.launch.py
```

## Control

The setup targets a differential drive robot. Typical teleop uses geometry_msgs/Twist on `/cmd_vel`.

- Option A: Use a keyboard teleop node (e.g., `teleop_twist_keyboard`) to publish to `/cmd_vel`.
- Option B: Write a simple publisher to `/cmd_vel` to move the robot.

Use `ros2 topic list` and `ros2 interface show geometry_msgs/msg/Twist` to confirm topics and message types at runtime.

## Robot description

- `urdf/robot.urdf.xacro` is the primary source; generate URDF via xacro if needed.
- Meshes are referenced from `meshes/` and define the visual/collision geometry.
- TF frames should include at least `base_link` and wheel links; verify with RViz TF display.

## Gazebo worlds

- `worlds/empty.sdf` — minimal world for quick startup
- `worlds/diff_drive.sdf` — example world configured for differential drive testing

You can switch worlds by editing the launch file or passing an argument (if implemented in `gazebo.launch.py`).

## ROS–Gazebo bridge

`config/ros_gz_bridge_config.yaml` defines the set of topics to bridge between ROS 2 and Gazebo Transport. Common bridges include clock, TF, joint states, laser/camera (if present), and `/cmd_vel`.

If you add new sensors or topics, extend this YAML accordingly and ensure the launch file loads it.

## Visualization

`rviz/display_config.rviz` provides a starting view of the robot and TF. Launch RViz via `rviz.launch.py` and adjust displays as needed.

## Development tips

- Use `--symlink-install` (already in build instructions) to iterate on Xacro, meshes, and launch files without reinstalling.
- After editing Xacro/URDF or meshes, you may need to relaunch RViz/Gazebo if models don’t refresh.
- Validate the description with `check_urdf` and visualize with `rviz`.

## Next steps (AI and control integration)

- Add a dedicated control node/package (e.g., PID velocity controller or model-based controller) subscribing to `/cmd_vel`.
- Add sensors (lidar/camera/IMU) in URDF and bridge them to ROS for perception.
- Integrate navigation (Nav2) or custom planners; provide TF tree and odometry.
- Add an AI module (e.g., perception with OpenVINO/ONNX/TensorRT) and wire it into the control loop.
- Create tests (launch and node-level) and CI to guard regressions.

## Troubleshooting

- If no topics are bridged, confirm `ros_gz_bridge` is installed and the YAML path used by the launch file is correct.
- If the robot is invisible in RViz, check fixed frame and TF availability; ensure URDF loads and `robot_state_publisher` is running (via the launch file).
- If Gazebo fails to start, verify Gazebo version compatibility with your ROS 2 distro.

## License

Licensed under the Apache License, Version 2.0. See `LICENSE` for details.

