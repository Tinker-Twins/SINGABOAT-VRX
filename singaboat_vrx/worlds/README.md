# SINGABOAT-VRX World Building Instructions

1. Change directory to `/singaboat_vrx/worlds` and create/edit a `<WORLD_NAME>.world.xacro` file.

2. Build the Gazebo world:
```bash
$ rosrun xacro xacro <SOURCE_FILE_NAME>.world.xacro > <TARGET_FILE_NAME>.world

$ rosrun xacro xacro open_ocean.world.xacro > open_ocean.world
$ rosrun xacro xacro sydney_regatta.world.xacro > sydney_regatta.world
$ rosrun xacro xacro vrx_course.world.xacro > vrx_course.world
```

3. The built world will be saved to `/singaboat_vrx/worlds` directory as a `<WORLD_NAME>.world` file.
