FROM clrnet:latest

ARG DEBIAN_FRONTEND=noninteractive

COPY ./src/detection /opt/detection
WORKDIR /opt/detection

# Install ROS 2 Foxy

RUN apt-get install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update
RUN apt-get install -y ros-foxy-ros-base python3-argcomplete

RUN echo "Hi :)"

ENTRYPOINT ["python3", "main.py"]