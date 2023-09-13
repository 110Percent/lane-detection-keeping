
## Document Structure

* What the project's about
	* Problem we're trying to solve
	* Options/alternatives (at least 2, preferably more)
		* Explanation of our choice
* Timeline of intended progress
* Resources/Purchases needed
* Roles of each person
* Budget/Engineering economics

According to [[Typical Proposal]]

## 2. Clear Statement of Objectives

- Software to detect lane data
- Software to translate lane data into movement commands
- Mechanism to react to movement commands
	- Robot / Simulation

## 3. Background

* Existing attempts to solve lane detection problem
* Methodologies, strengths, weaknesses
* Look for _common weaknesses_ in existing solutions and aim to tackle that
* Look for metrics/measurements for performance for lane detection
	* Quantitative measures important - Research papers especially good here

## 4. What We're Going to Do (System Structure)

* Assume road has no obstacles
### Detection
- **Specialists**: _Curtis_, _Ian_
* Start with prototype that accounts for visible, solid lines on either side of the car
* Account for various road conditions after
	* Road lines (colour/dash, bad lines)
	* Weather conditions (rain/snow)
	* Curbs
	* Changes in lighting (under a bridge? in a tunnel?)
* Account for road curvature
* Account for motion blur in camera
* Boils down to "where are any lanes in the car's line of sight"
* Python, OpenCV
* Neural Network? Explore possibility
	* Could be too complicated

### Keeping & Control
- **Specialists**: _Liam_, _Robert_, _Ian_
* Parses raw lane data from *Detection* module
* Make adjustment to steering to counteract lane drift
* Adjust speed to prevent car from running off road from inertia at turns
* Sends motion commands to *Movement* module
* Boils down to "where should the car go, based on lane data"

### Movement
- **Specialists**: _Robert_ 
* Accepts motion commands from K&C module and mechanically turns/accelerates test robot'
* Links K&C module with simulation environment for robot-free testing of methodology

### Working Environment
- **Specialists**: _Robert_, _Curtis_
- Laying groundwork for connection of ROS nodes
- Connection from software system to the environment, either simulation or real (robot)

## Desired Final Products

* "Real" Implementation
	* Lane detection and suggestion system that can be attached to a vehicle
		* Determines lane positions and provides feedback on how the car should move
	* Demonstrates that the system would work in a real scenario in real-time
* "Model" Implementation
	* System connected to a robot model car that navigates through a track / model road
	* Demonstrates that the system gives accurate commands to the movement module and that a vehicle would properly react to lane data

## 5. Relationship with Degree

All members are in software engineering
Agent-based programming
#### Detection
Computer vision, real-time computing
#### K&C
Data structures, real-time computing
#### Movement
Event-driven programming, embedded systems
#### Working Environment
Software Architecture/Design

## 6. Skills Required

Group has combined experience with Computer Vision, Robotics and Embedded Systems, as well as Software Architecture to make a coherent system that meets requirements.

## 7. Methods to Use
??

## 8. Proposed Timetable

- Figure out what we want to deliver and how we're going to break it down
- List stages of development
	- Robot
	- Video demo
	- Keep in mind project deadlines from website
- Start out with MVP that operates under ideal conditions
- After MVP is produced, iterate and expand scope to improve capabilities
	- Better detection
	- Speed control, recovery, lane merging
	- More focus on algorithmic improvement of the system
## 9. Risks & Mitigation Strategies

- Neural Networks / ML is too complicated - Detection, K&C
	- Fall back to algorithmic approach
- Scope Creep
	- Tight organisation of tasks; have it as organised as possible before starting
	- **Review tasks regularly to reassess deadlines realistically**

## 10. Special Components & Facilities

- Robot
	- Should be provided by Professor Shen as previously mentioned
- Test Track
	- Bristol board with drawn lines for lanes
	- Go into paved parking lot with tape for lanes
- Cameras for detection
- SBC for computation
	- Model may vary if we intend to do intensive computation, eg. ML
- Libraries/Software
	- Python/C++/OpenCV, other data processing libraries, PyTorch if we decide on ML
	- ROS for subsystem management and hardware integration
	- CARLA for simulation - has ROS bridge
		- Check for alternatives