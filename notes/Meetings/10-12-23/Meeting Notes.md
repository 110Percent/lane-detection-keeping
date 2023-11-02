## ClickUp
- to be used for task tracking
- Gantt Chart
- tasks have been mapped out
- MVP done by end of semester
- more tasks to be added for future development

## Keeping&Control Iterations
- Both Prof. Dansereau and Prof. Shen can give guidance for a lot of the control aspects of the system
- PID controller to start for MVP
	- good/easier initial setup
	- not the best solution, could have a lot of wobbling
	- not a complicated control system
	- basic has three parameters, tuned appropriately will do a reasonable job
- Model Predictive control (MPC) ----*will discuss later in the term*
	- Prof. Shen has a lot of experience with MPC and control systems in general
	- would be really cool to be able to implement
	- much more complicated, more factors to consider
		- Laplace transforms
		- transfer functions
	- need to create a model
		- look at experimental data
	- potential iteration for next term
	- Want to track where we've been and where we're going to provide better pathfinding
- May just need to tune well rather than MPC
	- can observe how well PID works and decide how to move forward

## ROS Boilerplate
- basic nodes setup
- send message to TestTopic
- Basic flow
	- Camera logs message, publishes to Detection, 
	- Detection logs message, publishes to K&C
	- K&C logs message, publishes to Movement
	- Movement logs message, interacts with hardware
- CARLA uses Ackerman steering instructions for vehicle movement
	- has specific format for instructions
		- will probably use this format for our movement message passing format
- CARLA ROSBridge not setup yet
- Camera data must be converted into movement of vehicle
	- transform between pixel distance and movement
	- different camera mounting positions
		- affect overall steering
		- wheels have to turn differently
		- potentially using config file
			- specify camera location etc.
			- creates baseline for how to base movement interpretations
	- Camera and vehicle may have deferent coordinate systems
		- same point in image, must be transformed from one reference frame to another
		- need vehicle and camera reference frames
		- also global frame but we don't need to consider for lane keeping
		- Acceleration is respect to vehicle ref frame
		- Camera image is with respect to camera ref frame
		- **configuring where the camera is mounted will specify the differences in the reference frames**
		- TF2
			- handles reference transformations, matrix multiplication
			- has functions to calculate these transformations
			- quite complicated, tools are available for doing this
			- tutorials available
			- requires config file - doesn't use URDF but package available to convert from URDF

## CARLA Hiccup
-  actual sim UI not showing up remotely when running CARLA
- program is running properly, not crashing
- Possible solutions
	- may just need to configure it differently
	- may be able to redirect display through cli parameter when running CARLA
		- instructions online
- Curtis will login to physical box to try fixing the setup
	- will get key from Prof. Dansereau

## Detection
- boilerplate setup so we can get moving on flow of detection processing
- should have updates with frequency >50, every 20ms at least
- setup Schema for messaging to K&C
	- lane data create our own datatype
		- list of points on coordinate system relative to car of center of line
		- K&C will determine curve for center of lane and make adjustments
	- control commands
		- twist datatype (geometry message)
			- linear and angular velocity
		- CARLA datatype for movement