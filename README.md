# SINGABOAT-VRX | Virtual RobotX (VRX) Competition

![Github Stars](https://badgen.net/github/stars/Tinker-Twins/SINGABOAT-VRX?icon=github&label=stars)
![Github Forks](https://badgen.net/github/forks/Tinker-Twins/SINGABOAT-VRX?icon=github&label=forks)
[![Docker Stars](https://badgen.net/docker/stars/tinkertwins/singaboat-vrx?icon=docker&label=stars)](https://hub.docker.com/r/tinkertwins/singaboat-vrx/)
[![Docker Pulls](https://badgen.net/docker/pulls/tinkertwins/singaboat-vrx?icon=docker&label=pulls)](https://hub.docker.com/r/tinkertwins/singaboat-vrx/)

<img src="https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/PNG/Team.png" width="1080">

**COMPETITION:** Virtual RobotX (VRX) Competition 2022

**TEAM NAME:** SINGABOAT-VRX

**INSTITUTE:** Nanyang Technological University, Singapore

**MEMBERS:** Tanmay Vilas Samak, Chinmay Vilas Samak and Chern Peng Lee

**ADVISOR:** Dr. Ming Xie

## DEPENDENCIES

1. Robot Operating System (ROS) - Tested with [ROS Noetic Ninjemys](http://wiki.ros.org/noetic) on [Ubuntu 20.04.4 LTS (Focal Fossa)](https://releases.ubuntu.com/20.04/).
2. [Virtual RobotX (VRX)](https://github.com/osrf/vrx) Simulation Environment - Included as `vrx` directory with this `SINGABOAT-VRX` repository.
3. Python3 Dependencies - Listed in `requirements.txt` file of `singaboat_vrx` ROS package. These can be simply installed using the following command:
    ```bash
    $ pip3 install -r requirements.txt
    ```

## SETUP

1. Clone this `SINGABOAT-VRX` repository:
    ```bash
    $ git clone https://github.com/Tinker-Twins/SINGABOAT-VRX.git
    ```
2. Make a directory `VRX_Workspace` to act as your catkin workspace for VRX Competition.
    ```bash
    $ mkdir -p VRX_Workspace/src/
    ```
3. Move the `SINGABOAT-VRX` repository to the source space (`src`) of your `VRX_Workspace`.
    ```bash
    $ mv ~/SINGABOAT-VRX ~/VRX_Workspace/src/
    ```
4. Build the packages within your `VRX_Workspace`.
    ```bash
    $ cd ~/VRX_Workspace
    $ catkin_make
    ```
5. Source the `setup.bash` file of your `VRX_Workspace`.
    ```bash
    $ echo "source /home/$USER/VRX_Workspace/devel/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```

## USAGE

1. Launch any VRX task simulation instance.
```bash
$ roslaunch vrx_gazebo <TASK_NAME>.launch urdf:=$(rospack find singaboat_vrx)/urdf/singaboat.urdf

$ roslaunch vrx_gazebo station_keeping.launch urdf:=$(rospack find singaboat_vrx)/urdf/singaboat.urdf
$ roslaunch vrx_gazebo wayfinding.launch urdf:=$(rospack find singaboat_vrx)/urdf/singaboat.urdf
$ roslaunch vrx_gazebo perception_task.launch urdf:=$(rospack find singaboat_vrx)/urdf/singaboat.urdf
$ roslaunch vrx_gazebo wildlife.launch urdf:=$(rospack find singaboat_vrx)/urdf/singaboat.urdf
$ roslaunch vrx_gazebo gymkhana.launch urdf:=$(rospack find singaboat_vrx)/urdf/singaboat.urdf
$ roslaunch vrx_gazebo scan_dock_deliver.launch urdf:=$(rospack find singaboat_vrx)/urdf/singaboat.urdf
```

2. Launch `singaboat_task_manager` to automatically identify the VRX task and execute the corresponding SINGABOAT-VRX solution algorithm.
```bash
$ roslaunch singaboat_vrx singaboat_vrx.launch

```

## DOCKER

The docker container image(s) containing all the source code as well as dependencies is available on [Docker Hub](https://hub.docker.com/r/tinkertwins/singaboat-vrx).

## VIDEOS

Demonstration videos are available on [YouTube](https://youtube.com/playlist?list=PLY45pkzWzH982DTT3n9avu5bGrd0cAcfZ).

| <a href="https://youtu.be/jI0Y9ovG-Dg"><img src="https://i.ytimg.com/vi/jI0Y9ovG-Dg/maxresdefault.jpg" width="500"></a> | <a href="https://youtu.be/x7_Z7U9vJPY"><img src="https://i.ytimg.com/vi/x7_Z7U9vJPY/maxresdefault.jpg" width="500"></a> |
|:-------------------------------------------------------------------------:|:-------------------------------------------------------------------------:|
| [Task Descriptions](https://youtu.be/jI0Y9ovG-Dg) | [Task Solutions](https://youtu.be/x7_Z7U9vJPY) |

## VRX COMPETITION 2022

[Our team worked meticulously](https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/PNG/Team%20Working%20on%20Tasks.png) to complete all the [6 tasks of VRX Competition 2022](https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/PNG/Tasks.png) and ultimately emerged as one of the winners of the challange, while also bagging several other special awards.

| ![Station-Keeping](https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/GIF/Task%201%20-%20Station-Keeping.gif) | ![Wayfinding](https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/GIF/Task%202%20-%20Wayfinding.gif) | ![Scene Perception](https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/GIF/Task%203%20-%20Scene%20Perception.gif) |
|:-----------------------------------------------:|:-------------------------------------------------:|:-----------------------------------------------:|
| [Station-Keeping Task](https://github.com/osrf/vrx/wiki/vrx_2022-station_keeping_task) | [Wayfinding Task](https://github.com/osrf/vrx/wiki/vrx_2022-wayfinding_task) | [Scene Perception Task](https://github.com/osrf/vrx/wiki/vrx_2022-perception_task) |
| ![Semantic Navigation](https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/GIF/Task%204%20-%20Semantic%20Navigation.gif) | ![Gymkhana Challenge](https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/GIF/Task%205%20-%20Gymkhana%20Challenge.gif) | ![Scan-Dock-Deliver](https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/GIF/Task%206%20-%20Scan-Dock-Deliver.gif) |
| [Semantic Navigation Task](https://github.com/osrf/vrx/wiki/vrx_2022-wildlife_task) | [Gymkhana Challenge Task](https://github.com/osrf/vrx/wiki/vrx_2022-gymkhana_task) | [Scan-Dock-Deliver Task](https://github.com/osrf/vrx/wiki/vrx_2022-scan_dock_deliver_task) |

The detailed scores and ranks of all the teams that qualified for finals of VRX Competition 2022 are available on [VRX GitHub Wiki](https://github.com/osrf/vrx/wiki/vrx_2022-phase3_results), and a summary of results and awards declared during the [VRX 2022 Award Ceremony](https://youtu.be/aVPmrvTCjpg) are available on [VRX Website](https://robotx.org/2022/05/04/vrx-2022-awards-and-final-standings/).

<img src="https://github.com/Tinker-Twins/SINGABOAT-VRX/blob/main/media/PNG/Awards.png" width="1080">
