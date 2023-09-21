## General Notes/Updates
- Carla is working and setup
- looked into performance metrics for detection and keeping subsystems
- JetAuto is available for testing (shared with other group)
- Carla - simulation rendering may be expensive
	- does not have to be real time
## Performance Metrics
- precision focused
### Detection
- Row Wise
	- scan lines across image, compare ground truths to lines from algorithm
	- seems like the preferred method
- Bezier curves
	- uses pixelized scanning and calculates a curve 
	- a bit more complicated
- TuSimple
	- large dataset of frames to compare
		- diverse set of images (different environments etc.)
	- uses row wise training for ML
- LLamas
	- Unsupervised Labeled Lane Marker Dataset
	- another option for training ML model
- Could require multiple methods used in combination
### ML 
- F1 scores

### PID Controllers
- control distance from center of lane, make sure vehicle stays in center of lane
	- estimate where the vehicle is based on image
	- determine desired position
	- may need other info that image data for knowing where the vehicle position is
		- could also be done with stereo camera
- Metrics:
	- rise time
	- overshoot
	- steady state error
	- settling time
- Tuning controllers
	- various methods

### Controlling the vehicle
- need to know where vehicle is, estimate where we are based on image
- calibration to determine 
	- how far apart cameras are
	- position on the vehicle cameras are
	- angles
	- FOV
	- lens distortion
	- colour correction
	- could be many factors to consider, for our project - don't need to cover everything
- Chessboard pattern for calibration
- URDF
	- robot has model of itself to know where parts are
	- configure camera position based on the model

## Previous Report
- many references for other reports
- good template and base info
- ubuntu 18.04 machine
	- good specs for running simulations
	- will gain access in the future for testing
	- may reinstall ubuntu with newer version and use docker for running CARLA