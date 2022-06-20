# SINGABOAT-VRX WAM-V Building Instructions

1. Change directory to `/singaboat_vrx/wamv` and edit the `component_config.yaml` and `thruster_config.yaml` files as required.

2. Launch the `singaboat_build_wamv.launch` file to build WAM-V as per the specifications of `component_config.yaml` and `thruster_config.yaml` files.
```bash
$ roslaunch <PACKAGE_NAME> <LAUNCH_FILE_NAME>.launch

$ roslaunch singaboat_vrx singaboat_build_wamv.launch
```

3. The built WAM-V will be saved to `/singaboat_vrx/urdf` directory as `singaboat.urdf` file by default (configurable through `singaboat_build_wamv.launch` file in `/singaboat_vrx/launch/tools` directory).
