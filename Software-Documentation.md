# Software Documentation

## Open Challenge
The goal of the open challenge is to make three laps around the track without touching any of the walls. For the open challenge, we use the camera as our only sensor. When one wall disappears, the robot turns that way. We use two ROIs (Region of Interests) to find the area of the left wall and the right wall. One is a wide rectangle on the left and the other is parallel. If there is a big enough difference between the area of both walls, the robot adjusts accordingly. We achieve the wall following by using PD (Proportional-Derivative) steering. The error is multiplied by our Kp value, then the last error is calculated then multiplied by our Kd value. These values added up tells the robot how much to steer. 

To make sure we stop after three laps, the robot detects the blue line. We use an ROI located in the middle of the screen to detect the blue line. For a complete detection, the robot first must see the blue line from a distance, then up close, and finally, the blue line must be out of the camera's sight. After twelve detections, the robot waits and then stops.

## Obstacle Management
The goal of the obstacle challenge is roughly the same as the open challenge with added obstacles, not only do you have to make three laps around the track without touching any of the walls you have to do it while avoiding preplaced traffic lights where the colour of the traffic lights determines what side you pass it on, for example if the colour is green then you pass on the left, and if the colour is red you'll have to pass on the right. In addition to the rules previously mentioned the robot also has a optional added difficulty of parking, where there is a parking lot added to the track in which the robot starts and has to end in while being parallel to the wall.

We finished the obstacle challenge by first making a wall follow code so that our robot would be able to run through the circuit, next we made it so that our camera would be able to tell what colour each traffic light was and therefore what side to pass it on, after our robot figures out what colour each traffic light is, it then turns so that the traffic light is far enough on the left or right side of the camera so that the robot wouldn't ram into it. After this it then tries to go back to the centre of the track and occasionally turns depending on if the traffic light is on the corner or not. Since wall follow with just a camera isn't very reliable on obstacle challenge due to the added traffic lights, we implemented a LiDAR sensor which helps keep track of walls put also helps with parking. The logic behind how we figure out the three laps are up is the same, we count how many blue lines we pass thus connecting to how many corners we've passed. 
### Pillar Detection
```ino
def detect_contours(self, img_lab, lower_lab, upper_lab, threshold = 500, draw_image = None, *, conditional=None, filterSolids=True, draw_bounding_box=True, draw=1, c_colour=None, b_colour=None):
    mask = cv2.inRange(img_lab, lower_lab, upper_lab)
    if conditional is not None:
      mask = cv2.bitwise_and(mask, conditional.astype(np.uint8) * 255)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if filterSolids: contours = list(filter(self.filterSolids, contours))
    center_x = 0
    center_y = 0
    bottom_y = 0
    cntList, MaxCnt, MaxCntArea, approx, bounding_box = processContours(contours, threshold, draw_image, draw_bounding_box=draw_bounding_box, draw=draw, c_colour=c_colour, b_colour=b_colour)
    if MaxCnt is not None:
      x, y, w, h = bounding_box
      center_x = x + w / 2
      center_y = y + h / 2
      bottom_y = y + h
    return cntList, MaxCnt, MaxCntArea, approx, bounding_box, center_x, center_y, bottom_y
```
### Parking Wall Detection
