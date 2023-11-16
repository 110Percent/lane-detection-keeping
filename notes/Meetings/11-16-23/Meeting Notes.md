## Plans for the week
- start on lane detection part of the project for next week
	- CNN
- next steps for K&C
	- create mock messages to send to K&C
	- may not need to worry about a lane
		- just have a current position and where we want to be
- Det-K&C schema
- create config file
- code cleanup for launch process
- Convert 1st person to bird's eye
	- maybe stereo cameras would help
		- may not actually give much depth information
			- look into paper's on this
		- try to get stereo working in CARLA
			- adjust ego vehicle config
	- research into existing systems
		- if they convert to bird's eye
			- how does this work
	- TF2
		- may have to delay until we have Det data that can be converted to bird's eye
## Running the project
### Created Dockerfile
- pretty simple
- single python package causing error (simple-pid)
	- currently downgrading to previous package to fix error
## run.sh
- spawns ego_vehicle
	- will be changed to use our own in the future
## Display
- currently showing edge detection
- should have options to display original preprocessed video feed
	- ideally have an overlay on the original feed
## Detection
- should identify adjacent lanes
	- help with merging
- probably don't need beyond the directly adjacent lanes (may not be able to see that anyway)
## Bezier curve modeling
- curve for each side of the lane
- maybe create a lane object that has two lane barriers
	- send a list of lanes from Det - K&C, current and adjacent