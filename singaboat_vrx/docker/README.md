1. Change directory to `$HOME/ROS_Workspaces/VRX/src` and build the docker image:
```bash
$ docker build --tag <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG> -f <PATH/TO/DOCKERFILE> .
$ docker build --tag tinkertwins/singaboat-vrx:v2022.1 -f vrx/singaboat_vrx/docker/Dockerfile .
```

2. Create a container with the image you created in the previous step:
```bash
$ docker run <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
$ docker run tinkertwins/singaboat-vrx:v2022.1
```

3. In a different terminal window, list all containers and take a note of desired CONTAINER ID:
```bash
$ docker ps -a
```

4. Commit changes to Docker Hub:
```bash
$ docker commit -m "<COMMIT MESSAGE>" -a "<FULL NAME>" <CONTAINER ID> <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
$ docker commit -m "SINGABOAT-VRX" -a "Tinker Twins" 35b286bf2b9b tinkertwins/singaboat-vrx:v2022.1
```

5. Login to Docker Hub:
```bash
$ docker login
```

6. Push container image to Docker Hub, once done, you should be able to see your repository on Docker Hub:
```bash
$ docker push <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
$ docker push tinkertwins/singaboat-vrx:v2022.1
```

7. Run the container to test it in interactive tty (i.e., `-it`) mode:
```bash
$ docker run -it --name <GIVE A CONTAINER NAME> <USERNAME>/<IMAGE_REPOSITORY_NAME>:<TAG>
$ docker run -it --name vrx tinkertwins/singaboat-vrx:v2022.1
```

8. In a different terminal window, execute a bash from within the container to access it:
```bash
$ docker exec -it <CONTAINER NAME> bash
$ docker exec -it vrx bash
```

9. Exit the bash:
```bash
$ exit
```

10. Kill the container:
```bash
$ docker kill <CONTAINER NAME>
$ docker kill vrx
```
