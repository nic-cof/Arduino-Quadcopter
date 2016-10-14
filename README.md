# Arduino-Quadcopter
Code from testing quadcopter components through completion.

Goal: 
Create an autonomous quadcopter using Arduino components. Create code for stable flight, sensors, and navigation.

Method: 
Code will be stored from each test to track progress. Use old code to learn from when new issues arise.
Thorough testing of individual components in quick iterations. Build from simple code and add more complex elements.
Combine individual codes into final flight code.

Progress:
(Oct 14, 2016)
Apparently didn't update this when I flew the quad. Several successful test flights manually trimming for level (tethered) flight. Switched over to autonomous control and immediately flipped over. One of the motor arms broke and due to the way I assembled V1.2 frame I needed to make an entirely new one. Chalk that one up to learning. New frame is completely designed, just needs to be printed and assembled; this time not using glue.

(AUG 18, 2016)
Finished quadcopter frame ysterday. Needed a purpose built frame to accurately test components. Uploaded first iteration IMU code. Uses complementary filter to prevent drift in IMU data. Need to adjust filter values as there is still drift. Bluetooth breakout arrives today.

(AUG 3, 2016) U2
Second code failed. Found instructable with ESC code, worked like a charm. Started with ESC_A. Added ESC_B, _C, and _D after.
All four can be controlled from a single input.
Next iteration will add bluetooth to remove tether.

(AUG 3, 2016)
Used old code from ESC and could not get it to function. Trying new method.

(AUG 2, 2016)
Using code found on internet froma similar project to test and configure ESCs. Previous attempt at this project was based off this code.
Fiddled with code for extensive time before researching alternative methods.
