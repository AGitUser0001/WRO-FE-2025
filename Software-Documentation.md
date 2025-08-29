# Software Documentation

## Open Challenge
The goal of the open challenge is to make three laps around the track without touching any of the walls. For the open challenge, we use the camera as our only sensor. When one wall disappears, the robot turns that way. We use two ROIs (Region of Interests) to find the area of the left wall and the right wall. One is a wide rectangle on the left and the other is symetrical. If there is a big enough difference between the area of both walls, the robot adjusts accordingly. We achieve the wall following by using PD (Proportional-Derivative) steering. The error is multiplied by our Kp value, then the last error is calculated then multiplied by our Kd value. These values added up tells the robot how much to steer. 

To make sure we stop after three laps, the robot detects the blue line. We use an ROI located in the middle of the screen to detect the blue line. For a complete detection, the robot first must see the blue line from a distance, then up close, and finally, the blue line must be out of the camera's sight. After twelve detections, the robot waits and then stops. 
## Obstacle Management

### Pillar Detection

### Parking Wall Detection
