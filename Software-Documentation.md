# Software Documentation

## Open Challenge
The goal of the open challenge is to make three laps around the track without touching any of the walls. For the open challenge, we use the camera as our only sensor. When one wall disappears, the robot turns that way. We use two ROIs (Region of Interests) to find the area of the left wall and the right wall. One is a wide rectangle on the left and the other is parallel. If there is a big enough difference between the area of both walls, the robot adjusts accordingly. We achieve the wall following by using PD (Proportional-Derivative) steering. The error is multiplied by our Kp value, then the last error is calculated then multiplied by our Kd value. These values added up tells the robot how much to steer. 

To make sure we stop after three laps, the robot detects the blue line. We use an ROI located in the middle of the screen to detect the blue line. For a complete detection, the robot first must see the blue line from a distance, then up close, and finally, the blue line must be out of the camera's sight. After twelve detections, the robot waits and then stops.

## Obstacle Challenge
The goal of the obstacle challenge is roughly the same as the open challenge with added obstacles, not only do you have to make three laps around the track without touching any of the walls you have to do it while avoiding preplaced traffic lights where the colour of the traffic lights determines what side you pass it on, for example if the colour is green then you pass on the left, and if the colour is red you'll have to pass on the right. In addition to the rules previously mentioned the robot also has a optional added difficulty of parking, where there is a parking lot added to the track in which the robot starts and has to end in while being parallel to the wall.

We finished the obstacle challenge by first making a wall follow code so that our robot would be able to run through the circuit, next we made it so that our camera would be able to tell what colour each traffic light was and therefore what side to pass it on, after our robot figures out what colour each traffic light is, it then turns so that the traffic light is far enough on the left or right side of the camera so that the robot wouldn't ram into it. After this it then tries to go back to the centre of the track and occasionally turns depending on if the traffic light is on the corner or not. Since wall follow with just a camera isn't very reliable on obstacle challenge due to the added traffic lights, we implemented a LiDAR sensor which helps keep track of walls put also helps with parking. The logic behind how we figure out the three laps are up is the same, we count how many blue lines we pass thus connecting to how many corners we've passed. 

## Obstacle Management

### Pillar Detection

### Parking Wall Detection
