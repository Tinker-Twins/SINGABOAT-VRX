# SINGABOAT-VRX Docker Instructions

#### Building Docker Image from the Dockerfile:

1. Change directory to `$HOME/VRX_Workspace/src` (catkin workspace for VRX Competition):
```bash
$ cd <PATH/TO/DIRECTORY> .
$ cd $HOME/VRX_Workspace/src
```

2. Build the docker image from the dockerfile:
```bash
$ docker build --tag <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG> -f <PATH/TO/DOCKERFILE> .
$ docker build --tag tinkertwins/singaboat-vrx:v2022.3 -f SINGABOAT-VRX/docker/Dockerfile .
```

#### Contaninerization and Pushing the Container to Docker Hub:

1. Run the image you created in the previous step inside a container:
```bash
$ docker run <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
$ docker run tinkertwins/singaboat-vrx:v2022.3
```

2. In a new terminal window, list all containers and make a note of the desired CONTAINER ID:
```bash
$ docker ps -a
```

3. Commit changes to Docker Hub:
```bash
$ docker commit -m "<COMMIT MESSAGE>" -a "<FULL NAME>" <CONTAINER ID> <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
$ docker commit -m "SINGABOAT-VRX" -a "Tinker Twins" 35b286bf2b9b tinkertwins/singaboat-vrx:v2022.3
```

4. Login to Docker Hub:
```bash
$ docker login
```

5. Push the container to Docker Hub, once done, you should be able to see your repository on Docker Hub:
```bash
$ docker push <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
$ docker push tinkertwins/singaboat-vrx:v2022.3
```

#### Running the Containerized Image:

1. Run the containerized image (follow either step 1.1 or 1.2):

	1.1. Run the containerized image in **headless mode** with a name (`--name`) and interactive tty (`-it`), and remove the container upon exiting (`--rm`):
	```bash
	$ docker run --rm -it --name <CONTAINER_NAME> <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
	$ docker run --rm -it --name singaboat_vrx tinkertwins/singaboat-vrx:v2022.3
	```

	1.2. Run the containerized image in **GUI mode** (requires a VNC viewer) with a name (`--name`) and interactive tty (`-it`), bind TCP port of host to port of the conatainer and publish it to the host system’s interfaces (`-p`), and remove the container upon exiting (`--rm`):
	```bash
	$ docker run --rm -it -p <HOST_PORT:CONTAINER_PORT> --name <CONTAINER_NAME> <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
	$ docker run --rm -it -p 5900:5900 --name singaboat_vrx tinkertwins/singaboat-vrx:v2022.3
	```
	
	Launch the VNC viewer (tested with [RealVNC Viewer](https://www.realvnc.com/en/connect/download/viewer/)) and configure the connection parameters (the password for establishing secure connection is `singaboat-vrx`):
	
	For running VNC viewer on the host machine:
	```bash
	$ vncviewer localhost:5900
	```
	
	For running VNC viewer on a remote machine:
	```bash
	$ vncviewer ip.address.of.host:5900
	```
	
	You can also configure the connection parameters via GUI of the VNC viewer application.

2. [Optional] Start additional bash session(s) within the container (each in a new terminal window):
```bash
$ docker exec -it <CONTAINER NAME> bash
$ docker exec -it singaboat_vrx bash
```

3. Once you are done with the intended job, exit the bash session(s):
```bash
$ exit
```

4. Kill the running container (required only if the containerized image is still running):
```bash
$ docker kill <CONTAINER NAME>
$ docker kill singaboat_vrx
```

5. Remove the container (required only if the containerized image is not run with the `--rm` option):
```bash
$ docker rm <CONTAINER NAME>
$ docker rm singaboat_vrx
```

#### Cleaning Up Docker Resources:

Running or caching multiple docker images or containers can quickly eat up a lot of disk space. Hence, it is always a good idea to frequently check docker disk utilization:
```bash
$ docker system df
```

In order to avoid utilizing a lot of disk space, it is a good idea to frequently purge docker resources such as images, containers, volumes and networks that are unused or dangling (i.e. not tagged or associated with a container). There are a number of ways with a lot of options to achieve this, please refer appropriate documentation. The easiest way (but a potentially dangerous one) is to use a single command to clean up all the docker resources (dangling or otherwise):
```bash
$ docker system prune -a
```
