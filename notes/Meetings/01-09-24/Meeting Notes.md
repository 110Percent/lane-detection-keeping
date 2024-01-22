## Keeping Model
- coordinate grid
- set of points to follow
- get lane data, with car at 0,0
	- determine movement commands
		- predict future movements
	- upon new data, restart prediction process for movement
### Correction directly from image
- have desired image and current, try to adjust to 
- just calculate what the error is and try to minimize
- when converting to coordinates, can have a lot of error propagation
	- inherent estimation of car location, need small error for this to work
	- can work for in between ticks
	- don't need bird's eye, need some math for accounting for perspective
- 50 Hz refresh
	- keep previous until new command
		- works if interval is small enough


## Oral Presentation
- roughly 10 mins per student
- profs + second reader
- questions at end
- assume no knowledge of topic
	- technical but understandable for other 4th year students
- make sure it is clear and well formatted
- lots of images/diagrams, no walls of text
- layout
	- overview of project
	- what we have done
		- demos
	- what to do next
### Demo
- can show car driving
	- interfacing with CARLA
- show diagrams of ROS setup
- show diagrams of Docker work
### Sections for each subsystem
