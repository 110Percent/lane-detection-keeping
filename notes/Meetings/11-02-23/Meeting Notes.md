## CARLA
### Maps
- lots available
- some probably available that are optimized for lane keeping
	- already found some good options to try
- can adjust environment to have trees, different lighting, imperfect conditions
### ROS Bridge not working with CARLA docker
- troubleshooting through this issue
- can maybe put ROS bridge in the container image
### ROS Default communication protocols
- must change settings to not use shared memory since we are using docker

## Movement
- changed architecture to include movement in K&C
- JetAuto does not use Ackerman steering
	- might be able to ask for a JetAcker
	- currently plan to create a translator... focusing on simulation with CARLA first (which uses Ackerman)
		- there are formulas available for the conversions

## PID Controller
- learning about setup
- lots of information available but taking time to go through
- figuring out how to configure everything
- GitHub repo available for similar implementation
- need
	- reference signal
	- measurements
	- feedback
- ways to handle diff frequency of camera and PID controller
	- EASIEST WAY: assume between the time everything remains constant
	- to improve: create model to predict movement in between
		- predict where the lane will be in future frames based on velocity etc of the vehicle and camera input

## Sunshine
- works really well
- can update config to allow multiple client connections
	- currently only allows 1

## Camera sensor color format
- most sensors don't use rgb in the actual camera
	- converted to rgb, the image file has the rgb data after its been converted
- Bayer pattern
- rggb