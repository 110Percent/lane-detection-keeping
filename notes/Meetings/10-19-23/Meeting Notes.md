Screen not connecting :(
- it was bad cable

## Proposal

### Notes from Profs
- based on headers, we have everything covered
- evaluation for each subsystem and overall evaluation is good
	- this is an important aspect
	- Where is data coming from?
		- for detection
			- TuSimple
			- Llamas
			- etc.
		- for keeping/movement
			- use papers with measurements
			- papers in control theory
			- convert theoretical problems in papers to our situation **(IMPORTANT)**
				- ex. for PID we need a reference, for us it is the center of the lane
- everything seems good, hard to say what is missing until it is fleshed out
	- we can email a draft of the report when we have it
		- send it early
	- any questions next week, just email
	- can potentially organize a meeting also (probably not necessary)

### Subsystems

### Detection

### Keeping

#### Movement Subsystem
- options for controller
	- PID Controller
		- just a few parameters
		- fast to run
	- MPC
		- must build a model
- how often to poll
	- controller should have updates with frequency >50 Hz, every 20ms at least
		- for low speeds try every 20 ms
		- **actual sensor system can have lower tick rate**
			- controller polls until an update arrives (or periodically if no update arrives) and reacts to it
			- probably only send updates when a change occurs, have a threshold for what requires corrective action

### Work Breakdown structure
- reformat diagrams
	- hard to read
- Gantt chart is set up
	- will be adjusted as we go
- Costs
	- parts etc. listed
	- table is good :)
- Risks
	- matrix set up in proposal doc
	- risk mitigation probably addressed throughout other sections of proposal
	- table is good (easier to read)


## Camera stuff
- low priority, focusing on using in simulation environment
- camera + lens works properly
- plays well with OpenCV
- seems all good so far
- lens may not have good enough FOV
	- can look into other lenses
- gets very hot
	- camera can handle decently high temps
	- need to adjust settings to keep temp low
		- operating mode, exposure time, fps
- think about what mode
	- continuous
	- pull data to topic
	- others...

## CARLA issues
- video choppy over RDP
	- might need to update firewall settings to allow for better streaming
		- send an email to support to ask to adjust settings
	- Sunshine
		- ports:
			- video: 47998
			- control: 47999
		- NVIDIA gamestream protocol
		- low latency video streaming
	- maybe use ssh tunneling to open the ports


## JetAuto
- uses linear and angular velocity
- may need to adjust movement commands to work with ackerman steering