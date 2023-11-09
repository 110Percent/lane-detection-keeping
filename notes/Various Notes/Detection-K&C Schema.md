## What will Detection gather?
- distance from center of the lane (DCL)
- lane shape? (straight/curved)
- Write all data to ROS topic

## What does K&C need?
- DCL
	- read from ROS topic

## JSON Message

### Sending DCL values
EXAMPLE POSSIBLE MESSAGE FORMAT:
```json
desiredDCL: 10.0
actualDCL: 7.0
```
- the desiredDCL will be calculated by Detection based on the size of the lane (which will be determined from the input image data)
- the actualDCL will be calculated by Detection based on where the vehicle is within the lane
	- must take into account where the camera is on the vehicle
- K&C will read the desired and actual DCL values and send correction commands

### Sending Curve of lane
```json
curve: <some equation to describe the lane>
velocity: 
angularVelocity:
```


### Combination of Curve and DCL values
```json
currentActualDCL: 7.0
curve: <curve described as list of points>
```


## Twist messages
https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
- sends linear and angular velocity
- may be more appropriate for sending from K&C to vehicle

