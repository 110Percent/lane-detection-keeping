# This Dockerfile is used to build the Docker container that runs the Detection subsystem.

# The Detection subsystem needs to run in its own container, as it relies on the dependencies provided by the NVIDIA CUDA image,
# which are not provided by the ROS Bridge container on which the rest of the system runs.

FROM clrnet:latest

ARG DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Foxy

RUN apt-get install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update
RUN apt-get install -y ros-foxy-ros-base python3-argcomplete ros-dev-tools ros-foxy-cv-bridge ros-foxy-vision-opencv

RUN python3 -m pip uninstall -y opencv-python
RUN python3 -m pip install opencv-python --upgrade

COPY ./ros2_ws /opt/lane-capstone
WORKDIR /opt/lane-capstone

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install"

CMD /bin/bash -c "bash ./run_detection.sh"