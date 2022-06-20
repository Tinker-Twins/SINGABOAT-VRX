# ROS Package of Team SINGABOAT-VRX for VRX Competition

<img src="https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/PNG/Team.png" width="1080">

**COMPETITION:** Virtual RobotX (VRX) Competition 2022

**TEAM NAME:** SINGABOAT-VRX

**INSTITUTE:** Nanyang Technological University, Singapore

**MEMBERS:** Tanmay Vilas Samak, Chinmay Vilas Samak and Chern Peng Lee

**ADVISOR:** Dr. Ming Xie

## USAGE

#### Simulate, Execute and Visualize (or not) SINGABOAT-VRX Solution(s) to VRX Tasks

Launching Gazebo, RViz and SINGABOAT-VRX solution(s) for VRX Competition.
```bash
$ roslaunch singaboat_vrx singaboat_<TASK_NAME>.launch

$ roslaunch singaboat_vrx singaboat_station_keeping.launch
$ roslaunch singaboat_vrx singaboat_wayfinding.launch
$ roslaunch singaboat_vrx singaboat_scene_perception.launch
$ roslaunch singaboat_vrx singaboat_semantic_navigation.launch
$ roslaunch singaboat_vrx singaboat_gymkhana_challenge.launch
$ roslaunch singaboat_vrx singaboat_scan_dock_deliver.launch
```

In order to run SINGABOAT-VRX solution(s) in headless mode (i.e., without visualizing Gazebo and RViz), make use of the `headless` argument.
```bash
$ roslaunch singaboat_vrx singaboat_<TASK_NAME>.launch headless:=true

$ roslaunch singaboat_vrx singaboat_station_keeping.launch headless:=true
$ roslaunch singaboat_vrx singaboat_wayfinding.launch headless:=true
$ roslaunch singaboat_vrx singaboat_scene_perception.launch headless:=true
$ roslaunch singaboat_vrx singaboat_semantic_navigation.launch headless:=true
$ roslaunch singaboat_vrx singaboat_gymkhana_challenge.launch headless:=true
$ roslaunch singaboat_vrx singaboat_scan_dock_deliver.launch headless:=true
```

#### Execute SINGABOAT-VRX Submission(s) to VRX Competition

1. Launching only SINGABOAT-VRX solution(s) for VRX Competition (a running VRX task simulation instance expected).
```bash
$ roslaunch singaboat_vrx singaboat_vrx.launch

```

OR

2. Launching only SINGABOAT-VRX solution for a particular VRX task (a running VRX task simulation instance expected).
```bash
$ roslaunch singaboat_vrx vrx_<TASK_NAME>.launch

$ roslaunch singaboat_vrx vrx_station_keeping.launch
$ roslaunch singaboat_vrx vrx_wayfinding.launch
$ roslaunch singaboat_vrx vrx_scene_perception.launch
$ roslaunch singaboat_vrx vrx_semantic_navigation.launch
$ roslaunch singaboat_vrx vrx_gymkhana_challenge.launch
$ roslaunch singaboat_vrx vrx_scan_dock_deliver.launch
```

#### Record SINGABOAT-VRX Solution(s) to VRX Tasks

Recording a SINGABOAT-VRX solution simulation instance.
```bash
$ roslaunch singaboat_vrx record_<TASK_NAME>.launch

$ roslaunch singaboat_vrx record_station_keeping.launch
$ roslaunch singaboat_vrx record_wayfinding.launch
$ roslaunch singaboat_vrx record_scene_perception.launch
$ roslaunch singaboat_vrx record_semantic_navigation.launch
$ roslaunch singaboat_vrx record_gymkhana_challenge.launch
$ roslaunch singaboat_vrx record_scan_dock_deliver.launch
```

#### Playback a Recorded SINGABOAT-VRX Solution(s) to VRX Tasks

Playing back a recorded SINGABOAT-VRX solution simulation instance.
```bash
$ roslaunch singaboat_vrx playback_<TASK_NAME>.launch

$ roslaunch singaboat_vrx playback_station_keeping.launch
$ roslaunch singaboat_vrx playback_wayfinding.launch
$ roslaunch singaboat_vrx playback_scene_perception.launch
$ roslaunch singaboat_vrx playback_semantic_navigation.launch
$ roslaunch singaboat_vrx playback_gymkhana_challenge.launch
$ roslaunch singaboat_vrx playback_scan_dock_deliver.launch
```

#### SINGABOAT-VRX Tools for Prototyping and Debugging Algorithms

Launching SINGABOAT-VRX tool(s) for prototyping and debugging VRX solution algorithms, which include:
1. `singaboat_build_wamv.launch` - Builds URDF model of SINGABOAT (WAM-V) using `component_config.yaml` and `thruster_config.yaml`.
2. `singaboat_vrx_world.launch` - Launches Gazebo Simulator instance with SINGABOAT (WAM-V) URDF model and all VRX task elements.
3. `singaboat_vrx_rviz.launch` - Launches RViz instance enabling real-time visualization of all WAM-V states and sensor data.
4. `singaboat_teleoperation.launch` - Launches `singaboat_teleoperation` node to teleoperate the WAM-V using standard PC keyboard.
5. `singaboat_inverse_kinematics.launch` - Launches `singaboat_inverse_kinematics` node along with `rqt_publisher` and `rqt_reconfigure` to work on or debug IK module of the WAM-V separately.
```bash
$ roslaunch singaboat_vrx singaboat_<TOOL_NAME>.launch

$ roslaunch singaboat_vrx singaboat_build_wamv.launch
$ roslaunch singaboat_vrx singaboat_vrx_world.launch
$ roslaunch singaboat_vrx singaboat_vrx_rviz.launch
$ roslaunch singaboat_vrx singaboat_teleoperation.launch
$ roslaunch singaboat_vrx singaboat_inverse_kinematics.launch
```
