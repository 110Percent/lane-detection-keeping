# Base image is ROS 1 since that doesn't have to be built from source for the bridge to be built
FROM ros:noetic

# Required to use the source command
SHELL ["/bin/bash", "-c"]

# Setup for building ROS 2 from source
RUN apt-get update && apt-get install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Add ROS repository to apt
RUN apt-get install software-properties-common -y
RUN add-apt-repository universe
RUN apt-get update && apt-get install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Add some ROS 2 dependencies
RUN apt-get update && apt-get install -y libbullet-dev python3-pip python3-pytest-cov ros-dev-tools
RUN python3 -m pip install -U argcomplete flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest
RUN apt-get install --no-install-recommends -y libasio-dev libtinyxml2-dev
RUN apt-get install --no-install-recommends -y libcunit1-dev python3-vcstool git

# Add the message packages to ROS 1 using apt
RUN apt-get install --no-install-recommends -y ros-noetic-ackermann-msgs

# Clone the ROS 2 source code into the container
RUN mkdir -p /ros2_foxy/src; cd /ros2_foxy; vcs import --input https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos src

# Clone the ackermann_msgs source code into the ROS 2 source code, meaning it will be built into ROS 2. The same repo is used for both ROS 1 and 2, so to get the ROS 2 source code, we must checkout the ros2 branch.
RUN cd /ros2_foxy/src/ros2/; git clone https://github.com/ros-drivers/ackermann_msgs.git; cd ackermann_msgs; git checkout ros2;

# Add ROS 2 dependencies from rosdep
RUN cd ros2_foxy; rosdep init; rosdep update; rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

# Build ROS 2, minus the bridge
RUN cd ros2_foxy; colcon build --symlink-install --packages-skip ros1_bridge 

# Build the ROS 2 bridge
RUN source /opt/ros/noetic/setup.bash; source /ros2_foxy/install/setup.bash; cd ros2_foxy; colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
