import cv2
import ros_robot_controller_sdk as rrc # pyright: ignore[reportMissingImports]
import time
import numpy as np
from picamera2 import Picamera2 # pyright: ignore[reportMissingImports]
from utils import processContours

ServoChannel = 4
MotorChannel = 1
ServoSpeed = 0.01
MotorTransitionSpeed = 0.1
MaxRightArea = 0
MaxLeftArea = 0
Kp = 0.1
Kd = 0.05
Ki = 0
i_error = 0
stMode = False
stMode_time = -1
stMode_dir = 0
error = 0
last_error = 0
threshold = 60
picam2 = Picamera2()
picam2.configure(
  picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    sensor={"output_size": (1640, 1232)}
  )
)
picam2.start()
board = rrc.Board()
motorPW = 1620
servoStraight = 1825
servoPW = servoStraight
steer = 0
rate_limit = 1/60
last_time = -1
last_servoPW = -1

lower_blue = np.array([35, 110, 0])
upper_blue = np.array([160, 255, 108])

turnCount = 0
last_turn_detection = -1
detected_turn = False
turn_limit = 12
  
def findContours(image, draw_image=None, *, draw=1, c_colour=None, b_colour=None):
  contours, _hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  cntList, MaxCnt, MaxCntArea, _approx, _bounding_box = processContours(contours, 200, draw_image, draw=draw, c_colour=c_colour, b_colour=b_colour)
  return cntList, MaxCnt, MaxCntArea

#Arm the Servo
board.pwm_servo_set_position(ServoSpeed, [[ServoChannel, servoPW]])
time.sleep(1)

board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPW]])                                  
while True:
  cur_time = time.time()
  if cur_time - last_time < rate_limit:
    time.sleep((last_time + rate_limit) - cur_time)
  cur_time = time.time()
  dt = cur_time - last_time
  last_time = cur_time

  img = picam2.capture_array()
  ROI_left = img[230:250, 0:300]
  ROI_right = img[230:250, 340:640]
  ROI_front = img[150:480, 70:570]

  ROI_left_grey = cv2.cvtColor(ROI_left, cv2.COLOR_BGR2GRAY)
  ROI_right_grey = cv2.cvtColor(ROI_right, cv2.COLOR_BGR2GRAY)
  ROI_front_lab = cv2.cvtColor(ROI_front, cv2.COLOR_BGR2LAB)
  cv2.rectangle(img, (0, 230), (300, 250), (255, 0, 0), 2)
  cv2.rectangle(img, (340, 230), (640, 250), (255, 0, 0), 2)
  cv2.rectangle(img, (70, 150), (570, 480), (255, 0, 255), 2)

  #cv2.imshow("gray", ROI_left_grey)
  ret, imgThresh = cv2.threshold(ROI_left_grey, threshold, 255, cv2.THRESH_BINARY_INV)
  #cv2.imshow("threshold", imgThresh)
  leftCntList, MaxLeftCnt, MaxLeftArea = findContours(imgThresh, ROI_left, c_colour=(255, 0, 0), b_colour=(0, 0, 255))

  #cv2.imshow("gray", ROI_right_grey)
  ret, imgThresh = cv2.threshold(ROI_right_grey, threshold, 255, cv2.THRESH_BINARY_INV)
  #cv2.imshow("threshold", imgThresh)
  rightCntList, MaxRightCnt, MaxRightArea = findContours(imgThresh, ROI_right, c_colour=(255, 0, 0), b_colour=(0, 0, 255))

  mask_blue = cv2.inRange(ROI_front_lab, lower_blue, upper_blue)
  cv2.imshow("Blue", mask_blue)
  blueCntList, MaxBlueCnt, MaxBlueArea = findContours(mask_blue, ROI_front)

  if turnCount < turn_limit and cur_time - last_turn_detection > (1 if detected_turn else 3):
    if MaxBlueCnt is not None:
      if not detected_turn:
        if MaxBlueArea > 500 and MaxBlueArea < 800:
          detected_turn = True
          last_turn_detection = cur_time
      else:
        if MaxBlueArea > 1000:
          detected_turn = False
          turnCount += 1
          last_turn_detection = cur_time
  cv2.putText(img, f"Turn detect: {int(detected_turn)}", (490, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
  cv2.putText(img, f"Turn count: {turnCount}", (490, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
  cv2.putText(img, f"Blue area: {MaxBlueArea}", (470, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

  error = MaxRightArea - MaxLeftArea
  if not stMode:
    i_error += error * dt
  cv2.putText(img, f"Left area: {MaxLeftArea}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
  cv2.putText(img, f"Right area: {MaxRightArea}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
  cv2.putText(img, f"Error: {error}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

  derivative = error - last_error
  last_error = error

  steer = Kp * error + Kd * derivative + Ki * i_error if abs(error) > 0 else 0
  steer = min(300, max(-300, steer))
  if stMode:
    steer = 200 * stMode_dir

  cv2.putText(img, f"Steer: {steer}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

  servoPW = servoStraight + int(steer)
  if servoPW != last_servoPW:
    board.pwm_servo_set_position(ServoSpeed, [[ServoChannel, servoPW]])
  last_servoPW = servoPW
  #print("servoPW =", servoPW)
  time.sleep(0.01)
  if (MaxLeftArea == 0) ^ (MaxRightArea == 0) and max(MaxLeftArea, MaxRightArea) > 1000 and not stMode:
    stMode = True
    stMode_time = time.time()
    stMode_dir = -1 if MaxRightArea == 0 else 1
    print("Steering Mode ON")
  if stMode and time.time() > stMode_time + 0.5 and \
     MaxLeftArea > 0 and MaxRightArea > 0 and (abs(error) < 500 or (abs(error) * -stMode_dir == error and abs(error) > 500)):
    stMode = False
    last_error = 0
    print("Steering Mode Off")
  print("error:", error)
  cv2.imshow("Camera", img)
  if cv2.waitKey(1) == ord('q') or (turnCount == turn_limit and cur_time - last_turn_detection > 3):
    motorPW = 1500
    board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPW]])
    break
     
cv2.destroyAllWindows()

