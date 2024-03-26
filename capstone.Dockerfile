FROM carla-ros-bridge:foxy

RUN /bin/bash -c "python3 -m pip install --force-reinstall 'simple-pid==1.0.1'"
RUN /bin/bash -c "python3 -m pip install --force-reinstall 'matplotlib'"

RUN mkdir -p /opt/lane-capstone
WORKDIR /opt/lane-capstone

COPY ./ros2_ws /opt/lane-capstone

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install"

COPY ./src/ros_entrypoint.sh /

CMD /bin/bash -c "echo 'Sleeping for a few seconds to allow CARLA to start...' && sleep 5 && bash ./run_main.sh"