# Docker Containers

Meant to provide basic docker containers for CI or development purposes. To use, make sure you have [Docker](https://docs.docker.com/get-docker/) installed. You then can pull the latest source or build image or build any version and run the image by following tag specific commands as described below:

## Source folder and tag
Downloads and builds rpi_pwm_hardware_interfaces into a workspace for development use exactly as is found. This is primarily used for development and CI of rpi_pwm_hardware_interface and related packages.

<!-- You can pull a prebuilt image with this docker tag: `ghcr.io/ros-controls/ros2_control_source`. -->

To build and run the container, execute the following code in a terminal:
```
docker build --tag rpi_pwm_hardware_interface:source --file .docker/Dockerfile .
docker run -it rpi_pwm_hardware_interface:source
```

If you are not familiar with docker, you'll want to consider how you want to maintain persistent data across container sessions. The above commands with start a **new** container and that container will persist as long as you don't remove it. If you want to return to it, you'll have to use the `docker start` command with the container id.

A better way to maintain your code changes across coding sessions is to use docker volumes or bind mounts. Web search those terms for more documentation but a simple example is provided below. First, you must build the container as shown in the first command above, then:

Bind mount example:
```
docker run -it --mount type=bind,source=/local/path/to/repo,destination=/root/ws_rpi_pwm_hardware_interface/src/<path to repo you want replaced> rpi_pwm_hardware_interface:source
```
Note: the above example replaces the containers contents with whatever you mount.

Volume example:
```
docker run -it --mount type=volume,source=name-of-volume,destination=/root/ws_rpi_pwm_hardware_interface/src/ rpi_pwm_hardware_interface:source
```
Note: this is probably the preferred solution as changes persist across rebuilds of container and doesn't require you to pull all repositories that you want to edit as they'll already be in the container.


### credit owed to ros2_control for the basic layout of the container
