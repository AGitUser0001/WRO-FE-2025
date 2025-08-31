# Software Documentation

## Open Challenge
The goal of the open challenge is to make three laps around the track without touching any of the walls. For the open challenge, we use the camera as our only sensor. When one wall disappears, the robot turns that way. We use two ROIs (Region of Interests) to find the area of the left wall and the right wall. One is a wide rectangle on the left and the other is parallel. If there is a big enough difference between the area of both walls, the robot adjusts accordingly. We achieve the wall following by using PD (Proportional-Derivative) steering. The error is multiplied by our Kp value, then the last error is calculated then multiplied by our Kd value. These values added up tells the robot how much to steer. 

To make sure we stop after three laps, the robot detects the blue line. We use an ROI located in the middle of the screen to detect the blue line. For a complete detection, the robot first must see the blue line from a distance, then up close, and finally, the blue line must be out of the camera's sight. After twelve detections, the robot waits and then stops.

## Obstacle Management
The goal of the obstacle challenge is roughly the same as the open challenge with added obstacles, not only do you have to make three laps around the track without touching any of the walls you have to do it while avoiding preplaced traffic lights where the colour of the traffic lights determines what side you pass it on, for example if the colour is green then you pass on the left, and if the colour is red you'll have to pass on the right. In addition to the rules previously mentioned the robot also has a optional added difficulty of parking, where there is a parking lot added to the track in which the robot starts and has to end in while being parallel to the wall.

We finished the obstacle challenge by first making a wall follow code so that our robot would be able to run through the circuit, next we made it so that our camera would be able to tell what colour each traffic light was and therefore what side to pass it on, after our robot figures out what colour each traffic light is, it then turns so that the traffic light is far enough on the left or right side of the camera so that the robot wouldn't ram into it. After this it then tries to go back to the centre of the track and occasionally turns depending on if the traffic light is on the corner or not. Since wall follow with just a camera isn't very reliable on obstacle challenge due to the added traffic lights, we implemented a LiDAR sensor which helps keep track of walls put also helps with parking. The logic behind how we figure out the three laps are up is the same, we count how many blue lines we pass thus connecting to how many corners we've passed. 
### Pillar Detection
```ino
class ObstacleChallengeProcess():
  lower_red = np.array([35, 150, 124])
  upper_red = np.array([140, 255, 255])
  lower_green = np.array([80, 0, 124])
  upper_green = np.array([190, 107, 255])
```
```ino
      # Red Detection
      _, _, MaxRedArea, _, _, red_x, _, red_y =self.detect_contours(ROI_front_LAB, self.lower_red, self.upper_red, draw_image=display_ROI_front, c_colour=(0, 0, 255), conditional=ROI_front_LAB[:, :, 1] > ROI_front_LAB[:, :, 2])
      
      # Green Detection
      _, _, MaxGreenArea, _, _, green_x, _, green_y =self.detect_contours(ROI_front_LAB, self.lower_green, self.upper_green, draw_image=display_ROI_front, c_colour=(0, 255, 0))
```





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

```ino
  lower_magenta = np.array([0, 155, 63])
  upper_magenta = np.array([140, 255, 130])
```

```ino
def parking(self, ROI_front_LAB, display_ROI_front, rw, rh, status, error_pillar, stopped, parking_detected, parking_side, last_parking_detect):
    limit = 500
    magenta1center_x = rw / 2
    magenta1center_y = rh / 2
    magenta1bottom_y = rh / 2
    magenta2center_x = rw / 2
    magenta2center_y = rh / 2
    magenta2bottom_y = rh / 2
    current_error = 0
```
```ino
    cntList, _, _, _, _, _, _, _ =self.detect_contours(ROI_front_LAB, self.lower_magenta, self.upper_magenta, limit, draw_image=display_ROI_front, draw=2)
    maxCnts = getMaxContours(cntList, 2)
    (maxCnt1, MaxMagentaArea1) = maxCnts[0] if len(maxCnts) > 0 else (None, 0)
    if maxCnt1 is not None:
      _approx, (x, y, w, h) = getBoundingBox(maxCnt1)
      magenta1center_x = x + w / 2
      magenta1center_y = y + h / 2
      magenta1bottom_y = y + h
    (maxCnt2, MaxMagentaArea2) = maxCnts[1] if len(maxCnts) > 1 else (None, 0)
    if maxCnt2 is not None:
      MaxMagentaArea2 = cv2.contourArea(maxCnt2)
      _approx, (x, y, w, h) = getBoundingBox(maxCnt2)
      magenta2center_x = x + w / 2
      magenta2center_y = y + h / 2
      magenta2bottom_y = y + h
```



## Our code explained

### Python libraries
For open challenge:
cv2  ros_robot_controller_sdk  time  numpy  picamera2  utils

For obstacle challenge:
ctypes  numpy  picamera3  time  ros_robot_controller_sdk  cv2  multprocessing  threading  queue  lidar  ObstacleChallengeProcess  utils

For ObstacleChallengeProcess:
numpy  math  cv2  time  queue  utils

For IMU:
subprocess  threading  math  multiprocessing

For LiDAR:
serial  struct  threading  multiprocessing  numpy  multiprocessing shared_memory  queue  time  cv2

For Utils:
Cv2  numpy  time

### LiDAR 
```ino
BAUD = 230400
PACKET_HEADER = 0x54
PACKET_LEN = 47

DATA_TYPE = np.float32
DATA_SIZE = np.dtype(DATA_TYPE).itemsize
DATA_RES = 3

NUM_POINTS = 360 * DATA_RES
NUM_VALUES = 6

class LiDAR:
  def __init__(self, shm_name='lidar', port="/dev/ttyAMA0"):
    self.shm_name = shm_name

    try:
      self.shared_memory = shared_memory.SharedMemory(create=True, size=NUM_POINTS*NUM_VALUES*DATA_SIZE, name=shm_name)
    except FileExistsError:
      self.shared_memory = shared_memory.SharedMemory(name=shm_name)

    self.lidar_array = np.ndarray((NUM_POINTS, NUM_VALUES), dtype=DATA_TYPE, buffer=self.shared_memory.buf)
    self.stopped = multiprocessing.Value('b', False)
    self.lock = multiprocessing.Lock()

    self.process = multiprocessing.Process(target=LiDARProcess, args=(\
      port, self.stopped, shm_name, self.lock), daemon=True)
    self.process.start()

    self.__visualizer_display_queue = None
    self.__visualizer_lidar_roi_queue = None
    self.__visualizer_thread = None
```



### Camera
### Steering
### Moving










