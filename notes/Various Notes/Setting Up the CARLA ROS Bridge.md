
## Docker

Install Docker on the target machine
[Ubuntu Instructions](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

Docker commands can only be run either as the root user or by someone in the `docker` group. Add your current user to the Docker group
```
sudo usermod -aG docker $USER
```
Log out and log back in to apply the change.

---

## CARLA

Pull the appropriate CARLA image from Docker Hub
```
docker pull carlasim/carla:0.9.13
```

Run this command to allow Docker to use the current X server:
```
xhost local:docker
```

❗ **Run this command to start CARLA in a Docker container**:
```
docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.13 ./CarlaUE4.sh
```

Note that this will start CARLA with "Epic" graphics quality. To reduce the quality and GPU load, run the command with `-quality-level=Low`

---

## CARLA ROS Bridge

Clone the [CARLA ROS Bridge](https://github.com/carla-simulator/ros-bridge) repository:
```
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git
```

Go to the docker folder:
```
cd ros-bridge/docker
```

Run the build script to generate a Docker image containing ROS and the bridge (you only need to do this once):
```
./build.sh
```

The ROS bridge is now ready to use within a Docker container. You can build off this image with a new Dockerfile or use the run script to access the existing image and use some of the demo programs included with the ROS bridge repository.

Open a Docker container using the run script:
```
./run.sh
```

The script should put you in a bash shell inside the container.

