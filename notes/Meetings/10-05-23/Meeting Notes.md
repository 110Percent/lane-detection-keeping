- can look into if a spare **jetson** or other controller is lying around
	- a group has one that we may be able to use
	- also consider what is optimal
- for our project we should only really worry about image data
	- GPS and lidar out of our scope
## camera/lens requirements
- sensor has gain setting to adjust to different lighting
	- can use same sensor in multiple environments
- basler apis for controlling camera
	- pypylon
	- also a c++ library
- configuring fstop
	- maybe value of ~10
	- not super precise
	- make out road details but not much else
- FOV
	- ~70-75 is probably fine
	- want overlap to get depth
	- want some peripherals
	- need to see ahead but not necessarily more
	- for rectangular sensor, FOV measured on diagonal not horizontal
	- MOUNT CAMERA DIAGONALLY FOR MAXIMUM FOV
- 2 cameras is fine for stereo
	- not trivial for configuring stereo
	- checkerboard config is generally for mono setup
	- looking for disparity estimation to determine depth
		- looking for distinct features in each image and comparing location
		- sift algorithm
	- OpenCV good for stereo calibration, Curtis has some experience from CV course


## Proposal
- Turkish proposal layout is very good!!
- Can apply it to requirements for our proposal
- how the **project applies to our degree** does not need to be very long
	- Essentially say we are writing code so it applies to software
### Non functional Requirements
- compliance
	- provincial guidelines
	- road laws/rules
	- what laws do we need to follow
	- Ontario regulations
		- we are level 2, legal with extensive testing
		- level 2 is still a human in control
		- talk about how often it works, level 5 is completely works by itself in any situation
		- **THIS IS GOOD FOR FINAL REPORT ALSO**
- Extensibility/Accessibility
	- should keep in mind to work with other components for a full autonomous car

### Constraints
- time
	- keep deadlines and scheduled tasks
	- modify if necessary
- all software students
	- don't worry about hardware
	- focus on what we know
- Budget
	- $500
	- borrow equipment from school

### Conceptual solutions
- review existing systems
	- use what companies have learned
- Issues with commercial versions
	- not robust
	- cannot handle many conditions
- academic approaches
	- gpu running ML models
		- not super feasible for a commercial car
			- environment is variable, temperature
			- need robust hardware
- For our implementation
	- have an on vehicle system
		- less powerful
		- no latency
	- have an off vehicle system
		- better hardware
		- can run ML model
		- must consider latency
	- want to make it as system versatile as possible
