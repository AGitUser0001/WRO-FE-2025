# Software Documentation

## Open Challenge
The goal of the open challenge is to make three laps around the track without touching any of the walls. For the open challenge, we use the camera as our only sensor. When one wall disappears, the robot turns that way. We use two ROIs (Region of Interests) to find the area of the left wall and the right wall. One is a wide rectangle on the left and the other is parallel. If there is a big enough difference between the area of both walls, the robot adjusts accordingly. We achieve the wall following by using PD (Proportional-Derivative) steering. The error is multiplied by our Kp value, then the last error is calculated then multiplied by our Kd value. These values added up tells the robot how much to steer. 

To make sure we stop after three laps, the robot detects the blue line. We use an ROI located in the middle of the screen to detect the blue line. For a complete detection, the robot first must see the blue line from a distance, then up close, and finally, the blue line must be out of the camera's sight. After twelve detections, the robot waits and then stops.

## Obstacle Management
The goal of the obstacle challenge is roughly the same as the open challenge with added obstacles, not only do you have to make three laps around the track without touching any of the walls you have to do it while avoiding preplaced traffic lights where the colour of the traffic lights determines what side you pass it on, for example if the colour is green then you pass on the left, and if the colour is red you'll have to pass on the right. In addition to the rules previously mentioned the robot also has a optional added difficulty of parking, where there is a parking lot added to the track in which the robot starts and has to end in while being parallel to the wall.

We finished the obstacle challenge by first making a wall follow code so that our robot would be able to run through the circuit, next we made it so that our camera would be able to tell what colour each traffic light was and therefore what side to pass it on, after our robot figures out what colour each traffic light is, it then turns so that the traffic light is far enough on the left or right side of the camera so that the robot wouldn't ram into it. After this it then tries to go back to the centre of the track and occasionally turns depending on if the traffic light is on the corner or not. Since wall follow with just a camera isn't very reliable on obstacle challenge due to the added traffic lights, we implemented a LiDAR sensor which helps keep track of walls put also helps with parking. The logic behind how we figure out the three laps are up is the same, we count how many blue lines we pass thus connecting to how many corners we've passed. 
### Pillar Detection
```py
class ObstacleChallengeProcess():
  lower_red = np.array([35, 150, 124])
  upper_red = np.array([140, 255, 255])
  lower_green = np.array([80, 0, 124])
  upper_green = np.array([190, 107, 255])
```
```py
      _, _, MaxRedArea, _, _, red_x, _, red_y =self.detect_contours(ROI_front_LAB, self.lower_red, self.upper_red, draw_image=display_ROI_front, c_colour=(0, 0, 255), conditional=ROI_front_LAB[:, :, 1] > ROI_front_LAB[:, :, 2])
      =
      _, _, MaxGreenArea, _, _, green_x, _, green_y =self.detect_contours(ROI_front_LAB, self.lower_green, self.upper_green, draw_image=display_ROI_front, c_colour=(0, 255, 0))
```





```py
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

```py
  lower_magenta = np.array([0, 155, 63])
  upper_magenta = np.array([140, 255, 130])
```

```py
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
```py
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
```py
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
1. Camera Initialization and Configuration:
The camera is initialized and configured in the cameraThread function. The Picamera2 library is used to interact with the camera.

```py
picam2 = Picamera2()
picam2.configure(
  picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    sensor={"output_size": (1640, 1232)}
  )
)
picam2.start()
```

This part of the code configures our video stream with a main output resolution of 640x480 pixels and a sensor output of 1640x1232 pixels. Next, picam2.start() begins the camera's video capture.

2. Image Capture and Processing
Within the main loop of the cameraThread, the camera continuously captures images resulting in our video.

```py
while not stopped.value:
  frame = picam2.capture_array("main")
  with frame_lock:
    global_frame = frame
```

The picam2.capture_array("main") function captures a single frame from the camera's main stream and stores it in the frame variable. This frame is then copied to the global_frame variable, which is shared with other parts of the code for further processing.

3. Sending Data
A specific region of interest is also extracted from the captured frame and sent to another process via a multiprocessing queue.

```py
roi_for_process = frame[front_coords[1] : front_coords[3], front_coords[0] : front_coords[2]]
if not roi_queue.full():
  roi_queue.put(roi_for_process)
```

This section of code takes a subsection of the captured video, the roi, and places it into the roi_queue. This allows another process (ObstacleChallengeProcess) to access this specific part of the image for its own analysis without needing access to the entire frame.

### Steering
The core of the steering control is in the main loop of the wallFollowThread. The code first processes images to determine a steering error and then uses a PID controller to calculate the steering value for the servo motor.

```py
def wallFollowThread(stopped, enter_parking, error_pillar, status):
  global wallFollow_display

  ServoChannel = 4
  MotorChannel = 1
  ServoSpeed = 0.01
  MotorTransitionSpeed = 0.1
  MaxRightArea = 0
  MaxLeftArea = 0
  Kp = 0.1
  Kd = 0.04
  Ki = 0
  i_error = 0
  stMode = False
  stMode_time = -1
  error = 0
  last_error = 0
  threshold = 60

  board = rrc.Board()
  motorPW = 1615
  parkMotorPW = 1625
  servoStraight = 1800
  servoPW = servoStraight
  steer = 0
  rate_limit = 1/60
  avg_dt = rate_limit
  last_time = -1
  last_servoPW = -1

  def setMotor(motorPos):
    board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPos]])

  def setServo(servoPos):
    board.pwm_servo_set_position(ServoSpeed, [[ServoChannel, servoPos]])
    
  parking = Parking()
  parking_stage = None

  direction = 0
  was_obstacle = False
  first_frame = True
  last_status = b"FORWARD"

  wall_detect_line = ((int(640 / 2), 220), (int(640 / 2), 360))
  wall_detect_timer = {}

  parking_detect_line_left = ((190, 190), (0, 320))
  parking_detect_line_front = ((320, 190), (320, 320))
  parking_detect_line_right = ((450, 190), (640, 320))

  lower_magenta = np.array([35, 110, 0])
  upper_magenta = np.array([160, 255, 108])

  def findContours(image, draw_image=None, *, draw=1, c_colour=None, b_colour=None):
    contours, _hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntList, MaxCnt, MaxCntArea, _approx, _bounding_box = processContours(contours, 200, draw_image, draw=draw, c_colour=c_colour, b_colour=b_colour)
    return cntList, MaxCnt, MaxCntArea

  setServo(servoPW)
  time.sleep(1)

  setMotor(1500)

  try:
    while not stopped.value:
      
      cur_time = time.time()
      if cur_time - last_time < rate_limit:
        time.sleep((last_time + rate_limit) - cur_time)
      cur_time = time.time()
      dt = cur_time - last_time
      if global_frame is None:
        continue
      last_time = cur_time
      avg_dt = (avg_dt * 0.95) + (dt * 0.05)

      with frame_lock:
        img = global_frame.copy()

      ROI_left = img[230:250, 0:300]
      ROI_right = img[230:250, 340:640]

      img_grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      _, img_grey_thresh = cv2.threshold(img_grey, threshold, 255, cv2.THRESH_BINARY_INV)
      ROI_left_thresh = img_grey_thresh[230:250, 0:300]
      ROI_right_thresh = img_grey_thresh[230:250, 340:640]
      cv2.rectangle(img, (0, 230), (300, 250), (255, 0, 0), 2)
      cv2.rectangle(img, (340, 230), (640, 250), (255, 0, 0), 2)


      _leftCntList, _MaxLeftCnt, MaxLeftArea = findContours(ROI_left_thresh, ROI_left, c_colour=(255, 0, 0), b_colour=(0, 0, 255))

      _rightCntList, _MaxRightCnt, MaxRightArea = findContours(ROI_right_thresh, ROI_right, c_colour=(255, 0, 0), b_colour=(0, 0, 255))

      current_error_pillar = -error_pillar.value
      if current_error_pillar != 0:
        if not was_obstacle:
          was_obstacle = True
        error = current_error_pillar
      else:
        if was_obstacle:
          was_obstacle = False
        error = MaxLeftArea - MaxRightArea

  
      if current_error_pillar == 0:
        steer = Kp * error + Kd * derivative + Ki * i_error if abs(error) > 0 else 0
        if stMode:
          steer = 200 * direction
      else:
        steer = current_error_pillar

      steer = min(300, max(-300, steer))
      servoPW = servoStraight + int(steer)
      
      if servoPW != last_servoPW:
        setServo(servoPW)
      last_servoPW = servoPW
      
  finally:

    setMotor(1500)
    parking.last_stage()
    print("WallFollow Thread Stopped")
```
The error variable is calculated as MaxLeftArea - MaxRightArea. If there's an obstacle, this value is overridden by error_pillar.value, which comes from the ObstacleChallengeProcess. This calculated error is then fed into a PD controller using the proportional gain Kp = 0.1 and derivative gain Kd = 0.04. The output of this controller is the steer value, which is then used to set the servo's pulse width. The setServo() function sends the final command to the physical servo, moving the wheels to steer the robot.


### Moving
The robot's forward and backward movement is controlled by the setMotor function, which sets the pulse width for the motor channel.

board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPos]]): This command is used to set the speed of the robot.
motorPW = 1615: This variable sets the standard pulse width for forward movement.
parkMotorPW = 1625: This variable sets the pulse width for forward movement during the parking sequence.
setMotor(1500): A pulse width of 1500 is used to stop the motor.
The if last_status != status: block controls the robot's direction and speed based on the status multiprocessing value (e.g., "FORWARD", "BACKWARD", "FORWARD_SLOW").

```py
def setMotor(motorPos):
    board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPos]])


cur_status = status.value
if last_status != status:
    if cur_status == b'BACKWARD':
        setMotor(1500)
        time.sleep(0.05)
        setMotor(1500 + (1500 - motorPW))
    elif cur_status == b'FORWARD':
        setMotor(motorPW)
    elif cur_status == b'FORWARD_SLOW':
        setMotor(motorPW - 5)
    elif cur_status == b'BACKWARD_SLOW':
        setMotor(1500)
        time.sleep(0.05)
        setMotor(1500 + (1500 - motorPW) + 5)
    last_status = status
```
This code block shows how the motor's position is adjusted to achieve forward, backward, and slow movement by changing the motorPos value passed to the setMotor function.







