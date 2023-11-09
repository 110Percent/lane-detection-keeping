## CARLA ROS Bridge
- container was available
- running smoothly
- going to make a note for instructions on how to setup CARLA and ROS Bridge with Docker so other groups can use it
	- for fresh Ubuntu install
		- install docker, pull images, run scripts, etc.
- GitHub repo has a lot of resources that will be helpful for when we write messages through ROS
## Det. - K&C Schema
- have the positioning converted to real world coordinates
- make all transformations in detection and send to K&C
- send curve equation vs set of points
	- pros and cons of both
### Curve fitting/region filling to lane
- detect where lane is
	- give curve fitting of center
	- give region of where lane is
		- could help for merging lanes
		- constructions cones as lane markings
		- if no markings, do we project where the lane should be
- maybe define a lane of where we think the lane should be, then detect where it should shift to
	- take hints to where the lane should be
### TF2 
- convert to real world positioning
- can do reference transformations

## Proposal feedback
- numbers in functional requirements need reasoning
- give evidence of why the numbers are good