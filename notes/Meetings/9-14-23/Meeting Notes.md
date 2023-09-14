# Connecting to the Screen
Some people have ports but there are issues with the actual thing, so Dansereau will check in with IT to have them properly configure this for use in future meetings. Getting resolved during the meeting? Somewhat works but also not at all so IT will need to come back.

# Meeting

## What are we writing out in the proposal?
- We have gone through the website for determining the necessary contents.
- Fourth Year info session - will go to.

## Report contents
- We want to have a software system that detects lanes on roads and determine the best course of action when it comes to steering and lane control. Demo in two ways
	- Attached to a real vehicle and can provide feedback based on the data in real time.
	- Work on a robot, we can show that the steering and physical feedback would work in a demonstration environment.
- Broke into four modules:
	- Detection: Gathering information from road. Use CV. Video data into a "raw" lane data.
	- Control module: Take data, make choices. Steering, control, etc.
	- Movement: Translates control into actual hardware level instructions. The abstraction layer with a robot.
		- Good for separation of environments
	- Environmental modules. Looking into using ROS for controlling subsets and whatnot, focus is on making sure that information being passed around is coherent.
		- Also for the simulation?
		- Dansereau: Not a lot of choice for other OS's. Most are ROS 1 or ROS 2. Some vehicles use QNX. Some use realtime versions of linux. ROS is clear winner, also because its open source. If a full size vehicle, we would probably consider QNX more but with a smaller situation the decision is clear.
		- Shen: If we are going to build a robot we probably need to think about the development board + Raspberry pi. ROS might work, look into. Also the power required for the project. More research into the hardware required in order to run what we need it to run.
		- Curtis: Brings up how we need to consider the use of ML in the project, and the hardware that can or cannot run this stuff.
		- If we need to do a remote thing, we may need to send the computations to another computer.
		- The lens Dansereau got might be useful. Unsure if software is necessary. For us, we need to check software requirements on any hardware that we want to get. Official pylon ROS driver for his lens for USB3. Again, we need to be very clear on the hardware specifications. (Basler Dart camera).
		- Getting a stereo camera. May be expensive to get is the problem, but it is probably required. Also recognize the dead zones that might exist on the stereo camera, and how to counteract that problem if the range is too short.
		- Curtis did do a demo and fount it to be a very good performance on his computer, and does show the ceiling of what we can do but just need to again make sure that the robot can either deal with the performance *or* we get it to outsource the CV and ML to another computer.
		- JetAuto robot is expensive and we would need to see its actual requirements.
	- Environment
		- We're abstracting the controls away from the environment its in, we can have a simulation for outside of the main project work.
		- Dansereau: Carla is a pain to install and get working. Don't use windows, use Ubuntu.
			- Can get us remote desktop access as well. Ubuntu 1804.
			- Carla to ROS bridges.
		- With respect to Roadrunner, do we have a license? We do, so no need to factor into the budget. ITS can hook us up with it.
		- Most stuff we're working with will be ROS 1, so we may need to use that instead and handle the Real Time requirements.
- Relation with degree:
	- Broke into modules.
	- Not hard to make these links.
- Skills required:
	- Lots of CV, robotics, algorithm thinking, etc.
- Methods to use:
	- We aren't sure on the expectations.
	- Lots of overlap and similarities. What do you think Dansereau?
	- Dansereau: We're going to use ROS to do this, stick a camera and drive a car. He doesn't like the proposal. Its good to break everything down in terms of the development, testing, implementation, etc.
- Timetable:
	- MVP by end of term
	- Then iterate in the winter term.
	- Get more timeline info with the info session for due dates.
	- Shen: Feedback control, PID controllers.
		- Should look more into PID controllers to figure out the good path.
	- Dansereau: Get a list of every task that we need to do in a very granular way and use that to determine the amount of time needed for each one.
- Performance: We need to make a section on this.
	- Shen: We might not have it right off the bat, but we do need to properly identify what performance is required for this system.
- Risks and mitigation:
	- We're very early in the conceptualization phase so we don't have a good foundation for potential risks and mitigation.
	- Neural networks might be a problem, offloading neural networks or not use it.
	- Scope creep is undesirable
	- Hardware selection might be incorrect, and then we lose out on our budget.
- Special components and facilities:
	- Robot: Find out what we're using for this, if its provided, if we need to but it and components.
		- Do we hijack the other team's robot?
		- Is there room for teams to share?
		- We are focused on software, and the robot is a way of showing off that.
		- Robot will likely exist and we can just borrow.
	- Test track: Demonstrate a robots capabilities on a physical track, parking lot, bristle board, etc.
	- Cameras
	- A board for running stuff.

## Agendas
- We don't need to make a formal itinerary and agenda before each meeting, we can if we want but its not necessarily required for this project.
- Just don't show up to meetings expecting it to be run for us.
- Don't want a scenario where nothing is being discussed. No meeting, but a short progress report.