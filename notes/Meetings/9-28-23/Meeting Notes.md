Computer details:
- Computer is avp-1.sce.carleton.ca
- SSH through port 20002
- lane is the username
- password is 2023-dansereau
- Can remote desktop through RDP.
- Carefully change things
- Only a 1 TB drive on it.

Cameras:
- Expected delivery date is a few days ago to next week. Maybe next meeting?
- Lenses in general: Three different kinds of lenses: Narrow, standard, fish-eye. Need to figure out which might be best to use.
- Fish eye: Unusable for what we want. Only important if we apply a lot of corrective transformations but that would also be a good amount of computation. Want to use if you need a full 360 degree view, not a focus on the front.
- The delivery one is a 49.1 degree FOV. A cheap test on. Not what we want.
- For lane detection, closer to 80 degrees.
- A LOT of lens categories. We need a good FOV. The cameras we have are a CS mount, which are less common. We could get a C mount but it would mean getting a short 5mm ring that can convert from a C to a CS mount.
	- Field of view, F stop, etc.
	- Test lens was from amazon.

Calibration:
- We found a couple resources on calibration.
- Some stuff set up with OpenCV to account for distortion in the image.
- Didn't do a stereo positioning of the camera.
- OpenCV should have a camera calibration auto calibration.
	- Calibration for camera is a big yes, but we can fix the robot height.
- However, the repo didn't include a CNN approach that we would be using.
- Any good implementation will use a mixture of both. Don't want to just be feeding raw image data into a net. 
- Camera calibration though the traditional approach is fine.
- Do we need huff transforms?
- Colourspace: The LAB colourspace
- Gert: Was using HSV but it was kind of picky with the edges.
- We can look at using a neural network to help out with it a bit more.

Plans:
- Robert will get ROSS on the machine for the Carla-ROSS bridge. This means setting it all up for docker integration.
- Curtis will look into the lens requirements.