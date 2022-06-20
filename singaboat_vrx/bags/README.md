# SINGABOAT-VRX | Bag Files

Record and playback bag files for all SINGABOAT-VRX simulation tasks with the `rosbag` command line interface (CLI). Note that the `record` launch files provided in the `singaboat-vrx` package will export recorded bag files to the `bags` directory and the `playback` launch files provided in the `singaboat-vrx` package will read recorded bag files from the `bags` directory.

***Note**: The recorded ROS bags have been removed to limit the size of this repository.*

## Recording New Bag Files

Launch SINGABOAT-VRX `record` launch files.
```bash
$ roslaunch singaboat_vrx record_<TASK_NAME>.launch

$ roslaunch singaboat_vrx record_station_keeping.launch
$ roslaunch singaboat_vrx record_wayfinding.launch
$ roslaunch singaboat_vrx record_scene_perception.launch
$ roslaunch singaboat_vrx record_semantic_navigation.launch
$ roslaunch singaboat_vrx record_gymkhana_challenge.launch
$ roslaunch singaboat_vrx record_scan_dock_deliver.launch
```

## Playing Back Recorded Bag Files

Launch SINGABOAT-VRX `playback` launch files.
```bash
$ roslaunch singaboat_vrx playback_<TASK_NAME>.launch

$ roslaunch singaboat_vrx playback_station_keeping.launch
$ roslaunch singaboat_vrx playback_wayfinding.launch
$ roslaunch singaboat_vrx playback_scene_perception.launch
$ roslaunch singaboat_vrx playback_semantic_navigation.launch
$ roslaunch singaboat_vrx playback_gymkhana_challenge.launch
$ roslaunch singaboat_vrx playback_scan_dock_deliver.launch
```
