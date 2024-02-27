## final report draft
- started, have outline in place
## building ros2
- just need to add Ackerman messages
## Det. - K&C messaging
- data serialized and sent to keeping system
- data then received and turned into "useful" data again
### next step: tf2 transform and then passed into K&C subsystem
## Control system using Carla waypoints
- consider which calculations for performance metrics
	- greatest distance from desired path, variance, etc.
- calculating tracking error - how do we do this?
	- must consider:
		- waypoints are discrete
		- actual path is continuous
## Visualizations
- show waypoints on overlay
- show desired path on overlay
- create separate ROS node for overlay display