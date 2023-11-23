## CNN
- use CLRNet
	- working on implementing it with our system
	- the abstraction they use is a bit funky
		- bad docs
	- wondering about detection speed
		- 20Hz is good
		- if low, maybe rely more on predictions
## Det-K&C 
### Row wise detection
- use more scan lines towards top of image
### Schema
- test_label_new.json format
## PID Controller
- using waypoints to adjust vehicle positioning
	- view converted from 1st person to bird's eye with grid points
- basic PID controller created
	- logic working
		- focus on polishing P value, then worry about I and D values
	- need to configure to reduce overshoot etc.
	- have to figure out the error based on projected course
	- P value is right now
		- ignores past and future
	- D value predicts what to do for future
		- sensitive to noise
		- perform fast fourier transform before passing data to PID controller
			- implement a low pass filter (D is calculated and passed through LPF)
				- any small amount of noise will greatly affect D term
					- we want the general trend for D, not every little jump back and forth
		- can simulate noise
	- can potentially model bezier curve to get desired trajectory and smooth data 
- starting off with steering and constant velocity
	- have CARLA handle velocity for now
	- once steering is done, velocity shouldn't be too difficult
## Progress Report
- outline what we have done
- progress on the project in the context of what the endpoint is
- note deviations from proposal
	- talk about what changes to the plan have been made
- talk about where the project needs to head for it to be successful