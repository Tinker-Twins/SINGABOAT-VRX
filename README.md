# SINGABOAT-VRX | Virtual RobotX (VRX) Competition

<a href="https://youtu.be/jI0Y9ovG-Dg"><img src="https://i.ytimg.com/vi/jI0Y9ovG-Dg/maxresdefault.jpg" width="1080" ></a>

**COMPETITION:** Virtual RobotX (VRX) Competition 2022

**TEAM NAME:** SINGABOAT-VRX

**INSTITUTE:** Nanyang Technological University, Singapore

**MEMBERS:** Tanmay Samak, Chinmay Samak and Chern Peng Lee

**ADVISOR:** Dr. Ming Xie

## DEPENDENCIES

1. Robot Operating System (ROS) - Tested with [ROS Noetic Ninjemys](http://wiki.ros.org/noetic) on [Ubuntu 20.04.4 LTS (Focal Fossa)](https://releases.ubuntu.com/20.04/).
2. [Virtual RobotX (VRX)](https://github.com/osrf/vrx) Simulation Environment - Included as `vrx` directory with this `SINGABOAT-VRX` repository.
3. Python3 Dependencies - Listed in `requirements.txt` file of this `SINGABOAT-VRX` repository. These can be simply installed using the following command:
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

Launch SINGABOAT-VRX solution(s) for VRX Competition.
```bash
$ roslaunch singaboat_vrx <TASK_NAME>.launch

$ roslaunch singaboat_vrx singaboat_station_keeping.launch
$ roslaunch singaboat_vrx singaboat_wayfinding.launch
$ roslaunch singaboat_vrx singaboat_scene_perception.launch
$ roslaunch singaboat_vrx singaboat_semantic_navigation.launch
$ roslaunch singaboat_vrx singaboat_gymkhana_challenge.launch
$ roslaunch singaboat_vrx singaboat_scan_dock_deliver.launch
```

## DOCKER

The docker container image(s) containing all the source code as well as dependencies is available on [Docker Hub](https://hub.docker.com/repository/docker/tinkertwins/singaboat-vrx).
