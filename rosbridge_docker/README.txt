This is a Docker container that builds ROS 2 and the ROS 1 bridge from source, allowing new message types to be added to the ROS 1 bridge. 

----------------------------------------------
Using the ROS 1 bridge

Instructions:
	1. From the rosbridge_docker directory, run the command 'source project_aliases'. This file contains a few aliases that are useful for building and using this Docker container.
	2. Run the command 'bridge_run' to launch the Docker container.
		NOTE: If there is already a ROS 1 core running on the device, skip steps 3-5.
	3. Run the command 'source /opt/ros/noetic/setup.bash'
	4. Run the command 'roscore' to start up the ROS 1 core. 
	5. Open a new terminal and follow step 1 again. Then run 'bridge_connect' to connect to the already Docker container.
	6. Run the command 'source /opt/ros/noetic/setup.bash' then 'source /ros2_foxy/install/setup.bash'
	7. Run the command 'export ROS_MASTER_URI=http://localhost:11311'
	8. Run ros2 run ros1_bridge dynamic_bridge
Upon completing these steps, the ROS 1 bridge should be running.


----------------------------------------------
Adding a new message type to the ROS 1 bridge

Instructions:
	1. Find the ROS 1 apt package for the message type you wish to add to the bridge and add it on line 27 of the Dockerfile (the line that installs ros-noetic-ackermann-msgs)
	2. Find the source repository for the ROS 2 message type you wish to add to the bridge, and clone it into ros2_foxy/src/ros2. 
		NOTE: Make sure that you are cloning the ROS 2 version. In some cases (such as with ackermann_msgs) they are part of the same git repository, and the master branch is typically for ROS 1. For ackermann_msgs, after cloning the repo we had to checkout the ROS 2 branch.
	3. Run the command 'source project_aliases' from the rosbridge_docker directory, then run the command 'bridge_build'. This is just an alias for Docker build with some extra arguments.
		NOTE: This will likely take a long time (about an hour for me, although my laptop isn't the fastest out there).
	4. From here, follow the instructions on using the bridge.

Limitations:
	The message type must be available in an apt repository for ROS 1. This limitation could be removed in the future by modifying the Dockerfile to build ROS 1 from source, but this was developed to use ackermann_msgs, which are not part of ROS by default but are available for installation via apt, so we did not bother with the extra headache.


----------------------------------------------
Resources:
	https://github.com/ros2/ros1_bridge
