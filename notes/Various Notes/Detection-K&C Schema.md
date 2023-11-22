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


# Chosen Schema
- taken from TuSimple 
	- https://www.kaggle.com/datasets/manideep1108/tusimple/
- "lanes" represents the detected lane markings(lines)
	- each line is represented by a list of x coordinates
- "h_samples" represents the horizontal lines used in the row-wise analysis
- "raw_file" is the raw image file
## Example
```json
{
"lanes": 
	[[-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 648, 636, 626, 615, 605, 595, 585, 575, 565, 554, 545, 536, 526, 517, 508, 498, 489, 480, 470, 461, 452, 442, 433, 424, 414, 405, 396, 386, 377, 368, 359, 349, 340, 331, 321, 312, 303, 293, 284, 275, 265, 256, 247, 237, 228, 219], 
		 
	[-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 681, 692, 704, 716, 728, 741, 754, 768, 781, 794, 807, 820, 834, 847, 860, 873, 886, 900, 913, 926, 939, 952, 966, 979, 992, 1005, 1018, 1032, 1045, 1058, 1071, 1084, 1098, 1111, 1124, 1137, 1150, 1164, 1177, 1190, 1203, 1216, 1230, 1243, 1256, 1269], 
		 
	[-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 713, 746, 778, 811, 845, 880, 916, 951, 986, 1022, 1057, 1092, 1128, 1163, 1198, 1234, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2], 
		 
	[-2, -2, -2, -2, -2, -2, -2, -2, -2, -2, 754, 806, 858, 909, 961, 1013, 1064, 1114, 1164, 1213, 1263, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2]],

"h_samples": 
	[160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710], 
 
"raw_file": 
	"test_set/clips/0530/1492626760788443246_0/20.jpg"
}
```
