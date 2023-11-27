#!/bin/bash

function cleanup() {
	echo ""
	echo stopping docker containers:
	docker stop carla_docker
	docker stop ros_docker
	echo removing docker container:
	docker rm carla_docker
	docker rm ros_docker
}
trap cleanup EXIT

echo starting docker containers, this may take a moment
echo ""

xhost local:docker &
docker run --name carla_docker -d --privileged --gpus=all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh -RenderOffScreen &

sleep 3

docker run --name ros_docker -d --net=host --gpus=all -e DISPLAY=$DISPLAY -it lane &

# Sleep until ctrl+c
sleep infinity
