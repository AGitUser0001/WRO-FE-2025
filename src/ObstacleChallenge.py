import threading
import multiprocessing
import cv2
import ros_robot_controller_sdk as rrc # pyright: ignore[reportMissingImports]
import time
from picamera2 import Picamera2 # pyright: ignore[reportMissingImports]
import numpy as np
import ctypes
import queue
from lidar import LiDAR
from ObstacleChallengeProcess import ObstacleChallengeProcess
from utils import processContours, getCollisions, get_timer, set_timer
from imu_scan import imu_value
#pylint: disable=redefined-outer-name
# --- Thread 1: Camera Capture ---
def cameraThread(stopped, roi_queue):
  global global_frame
  picam2 = Picamera2()

  picam2.configure(
    picam2.create_video_configuration(
      main={"size": (640, 480), "format": "RGB888"},
      sensor={"output_size": (1640, 1232)}
    )
  )
  picam2.start()

  while not stopped.value:
    frame = picam2.capture_array("main")
    with frame_lock:
      global_frame = frame
    
    # --- Send ROI to Obstacle Process ---
    # Extract the ROI needed for the other process and put it in the queue
    roi_for_process = frame[front_coords[1] : front_coords[3], front_coords[0] : front_coords[2]]
    if not roi_queue.full():
      roi_queue.put(roi_for_process)

  picam2.stop()
  print("Camera Thread Stopped")


# --- Thread 2: Wall Following and Motor Control ---
def wallFollowThread(stopped, error_pillar, obstacle_status):
  global wallFollow_display

  ServoChannel = 4
  MotorChannel = 1
  ServoSpeed = 0.01
  MotorTransitionSpeed = 0.1
  MaxRightArea = 0
  MaxLeftArea = 0
  Kp = 0.1
  Kd = 0.03
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
  servoStraight = 1825
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

  def parking(n = 3):
    s_error = last_error / (abs(last_error) + 1e-8)
    direction = int(s_error / abs(s_error))
    OUT = servoStraight + (400 * direction)
    IN = servoStraight - (400 * direction)
    setServo(servoStraight)
    setMotor(1500)
    time.sleep(0.15)
    for i in range(n):
      setServo(OUT)
      setMotor(parkMotorPW)
      time.sleep(0.4)
      setServo(IN)
      setMotor(1500 + (1500 - parkMotorPW))
      time.sleep(0.4)
      if i == n - 1: break
    return direction

  direction = 0
  was_obstacle = False
  first_frame = True
  last_status = b"FORWARD"

  lidar_array, lidar_shm = lidar.get_array_copy()
  lidar_roi_left = ((-750, -25), (-100, 25))
  lidar_roi_right = ((100, -25), (750, 25))
  _, lidar_roi_queue = lidar.get_visualizer()
  lidar.add_roi(lidar_roi_queue, (0, 255, 0), *lidar_roi_left)
  lidar.add_roi(lidar_roi_queue, (0, 255, 0), *lidar_roi_right)
  
  wall_detect_line = ((int(640 / 2), 220), (int(640 / 2), 360))
  wall_detect_timer = {}

  def findContours(image, draw_image=None, *, draw=1, c_colour=None, b_colour=None):
    contours, _hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cntList, MaxCnt, MaxCntArea, _approx, _bounding_box = processContours(contours, 200, draw_image, draw=draw, c_colour=c_colour, b_colour=b_colour)
    return cntList, MaxCnt, MaxCntArea

  #Arm the Servo
  setServo(servoPW)
  time.sleep(1)

  setMotor(motorPW)
  
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

      #cv2.imshow("threshold", ROI_left_thresh)
      _leftCntList, _MaxLeftCnt, MaxLeftArea = findContours(ROI_left_thresh, ROI_left, c_colour=(255, 0, 0), b_colour=(0, 0, 255))

      #cv2.imshow("threshold", ROI_right_thresh)
      _rightCntList, _MaxRightCnt, MaxRightArea = findContours(ROI_right_thresh, ROI_right, c_colour=(255, 0, 0), b_colour=(0, 0, 255))
        
      _LeftDist = -LiDAR.get_angle_median(lidar_array, 270 - 4, 270 + 5)[3]
      _RightDist = LiDAR.get_angle_median(lidar_array, 90 - 5, 90 + 4)[3]
      _FrontDist = LiDAR.get_angle_median(lidar_array, 0 - 5, 0 + 5)[4]

      current_error_pillar = -error_pillar.value
      if current_error_pillar != 0:
        if not was_obstacle:
          was_obstacle = True
        error = current_error_pillar
      else:
        if was_obstacle:
          was_obstacle = False
        error = MaxLeftArea - MaxRightArea
        
      num_collisions = getCollisions(img_grey_thresh, *wall_detect_line)
      will_collide_with_wall = num_collisions > 20
      
      wall_detect_timer_res, wall_detect_timer_high = get_timer(wall_detect_timer, 3, 0.2)
      if will_collide_with_wall and wall_detect_timer_res and not wall_detect_timer_high:
        set_timer(wall_detect_timer, True)
      if wall_detect_timer_res and wall_detect_timer_high:
        set_timer(wall_detect_timer, False, will_collide_with_wall)
        if will_collide_with_wall:
          stMode = True
          stMode_time = time.time()
          print("Steering Mode ON")
      if not wall_detect_timer_res and not wall_detect_timer_high:
        cv2.line(img, *wall_detect_line, (0, 0, 255), thickness=1)
      else:
        cv2.line(img, *wall_detect_line, (255, 0, 0) if will_collide_with_wall else (0, 255, 0), thickness=1)

      cv2.putText(img, f"Left area: {MaxLeftArea}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
      cv2.putText(img, f"Right area: {MaxRightArea}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
      cv2.putText(img, f"Error (-left +right): {error}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
      
      if obstacle_status.value.startswith(b"FORWARD"):
        error = -error

      if not stMode:
        i_error += error * dt

      derivative = error - last_error
      last_error = error
      
      cv2.putText(img, f"Error (-right +left): {error}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
      cv2.putText(img, f"Derivative: {derivative}", (10, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
      cv2.putText(img, f"Last error: {last_error}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

      if current_error_pillar == 0:
        steer = Kp * error + Kd * derivative + Ki * i_error if abs(error) > 0 else 0
        if stMode:
          steer = 200 * direction
      else:
        steer = current_error_pillar

      steer = min(300, max(-300, steer))
      cv2.putText(img, f"Steer: {steer}", (10, 175), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
      cv2.putText(img, f"FPS: {1 / avg_dt}", (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

      servoPW = servoStraight + int(steer)
      if servoPW != last_servoPW:
        setServo(servoPW)
      last_servoPW = servoPW
      #print("servoPW =", servoPW)
      time.sleep(0.01)
      if (MaxLeftArea == 0) ^ (MaxRightArea == 0) and max(MaxLeftArea, MaxRightArea) > 1000 and not stMode:
        time.sleep(0.15)
        if (MaxLeftArea == 0) ^ (MaxRightArea == 0) and max(MaxLeftArea, MaxRightArea) > 1000:
          stMode = True
          stMode_time = time.time()
          print("Steering Mode ON")
      if stMode and time.time() > stMode_time + 0.5 and \
         abs((imu_value.value % 90) - 45) <= 15:
      #  MaxLeftArea > 0 and MaxRightArea > 0 and (abs(error) < 9999 or (abs(error) * -direction == error and abs(error) > 9999)):
        stMode = False
        last_error = 0
        print("Steering Mode Off")
      #print("error:", error)

      status = obstacle_status.value
      if last_status != status:
        if status == b'BACKWARD':
          setMotor(1500)
          time.sleep(0.05)
          setMotor(1500 + (1500 - motorPW))
        elif status == b'FORWARD':
          setMotor(motorPW)
        elif status == b'FORWARD_SLOW':
          setMotor(motorPW - 5)
        elif status == b'BACKWARD_SLOW':
          setMotor(1500)
          time.sleep(0.05)
          setMotor(1500 + (1500 - motorPW) + 5)
        last_status = status

      with frame_lock:
        wallFollow_display = img
      if first_frame:
        direction = parking()
        setMotor(motorPW)
        first_frame = False

  finally:
    lidar_shm.close()
    # Graceful shutdown of motors
    setMotor(1500)
    print("WallFollow Thread Stopped")

# --- Process 1: Obstacle Detection ---

if __name__ == "__main__":
  # --- Shared Memory and Flags ---
  # For communication between processes
  stopped = multiprocessing.Value("b", False)
  error_pillar = multiprocessing.Value("d", 0.0)
  obstacle_status = multiprocessing.Value(ctypes.c_char_p, b"FORWARD")
  roi_queue = multiprocessing.Queue(maxsize=1)  # Queue to send ROI
  obstacle_display_queue = multiprocessing.Queue(maxsize=1)  # Queue to receive display data

  # For communication between threads
  global_frame = np.zeros((480, 640, 3), dtype=np.uint8)
  frame_lock = threading.Lock()
  wallFollow_display = np.zeros((480, 640, 3), dtype=np.uint8)

  print("Initializing...")

  lidar = LiDAR()
  # --- Create and Start Threads and Processes ---
  p_obstacle = multiprocessing.Process(
    target=ObstacleChallengeProcess,
    args=(stopped, error_pillar, roi_queue, obstacle_display_queue, obstacle_status),
    daemon=True
  )
  t_camera = threading.Thread(target=cameraThread, args=(stopped, roi_queue))
  t_wallFollow = threading.Thread(
    target=wallFollowThread,
    args=(stopped, error_pillar, obstacle_status),
  )

  print("Starting Process and Threads...")
  p_obstacle.start()
  t_camera.start()
  t_wallFollow.start()

  # --- Main Display Loop ---
  last_display_time = 0
  display_interval = 1 / 30  # 30 FPS
  #front_coords = (70, 140, 570, 360)
  front_coords = (0, 140, 640, 360)
  orig_front_coords = front_coords
  
  visualizer, lidar_roi_queue = lidar.get_visualizer()
  try:
    while True:
      now = time.time()
      if (now - last_display_time) >= display_interval:
        with frame_lock:
          # Make copies for safe display
          wf_image = wallFollow_display.copy()

          # Create obstacle display image
          oc_image = global_frame.copy()
        
        try:
          visualizer_image = visualizer.get_nowait()
          cv2.imshow("Visualizer", visualizer_image)
        except queue.Empty:
          pass

        # Try to get obstacle display data
        try:
          display_data = obstacle_display_queue.get_nowait()
          # Update the front ROI with obstacle detection visualization
          roi_display = display_data["roi"]
          oc_image[
            front_coords[1] : front_coords[3],
            front_coords[0] : front_coords[2],
          ] = roi_display

          # Add text overlay with detection info
          cv2.putText(
            oc_image,
            f"Red Area: {display_data['MaxRedArea']:.0f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
          )
          cv2.putText(
            oc_image,
            f"Green Area: {display_data['MaxGreenArea']:.0f}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
          )
          cv2.putText(
            oc_image,
            f"Error: {display_data['error_pillar']:.1f}",
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 255),
            2,
          )
          cv2.putText(
            oc_image,
            f"Steer Count: {display_data['steerCount']:.0f}",
            (493, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 0),
            2,
          )
          cv2.putText(
            oc_image,
            f"Distance: {display_data['distance']:.2f}",
            (493, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
          )
          cv2.putText(
            oc_image,
            f"Offset: {display_data['offset']:.2f}",
            (493, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
          )
          cv2.putText(
            oc_image,
            f"FPS: {1/display_data['avg_dt']:.2f}",
            (493, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
          )

          if display_data['parking']:
            cv2.putText(
              oc_image,
              f"Parking State: {display_data['parking_detected']:.1f}",
              (10, 420),
              cv2.FONT_HERSHEY_SIMPLEX,
              0.6,
              (255, 0, 255),
              2,
            )
            if display_data['parking_detected'] >= 1:
              front_coords = (0, front_coords[1], 640, front_coords[3])
            else:
              front_coords = orig_front_coords

          cv2.rectangle(
            oc_image,
            (front_coords[0], front_coords[1]),
            (front_coords[2], front_coords[3]),
            (0, 255, 0),
            2,
            )
          # Draw center lines for reference
          if display_data["redcenter_x"] > 0:
            center_x = front_coords[0] + int(display_data["redcenter_x"])
            cv2.line(
              oc_image,
              (center_x, front_coords[1]),
              (center_x, front_coords[3]),
              (0, 0, 255),
              2,
            )
          if display_data["greencenter_x"] > 0:
            center_x = front_coords[0] + int(display_data["greencenter_x"])
            cv2.line(
              oc_image,
              (center_x, front_coords[1]),
              (center_x, front_coords[3]),
              (0, 255, 0),
              2,
            )

          cv2.imshow("Obstacle Challenge", oc_image)
        except queue.Empty:
          # No obstacle data available
          pass

        cv2.imshow("Wall Following", wf_image)

        last_display_time = now

      if cv2.waitKey(1) == ord("q") or stopped.value:
        print("Stopping...")
        stopped.value = True
        break
  finally:
    # --- Cleanup ---
    p_obstacle.join()
    t_wallFollow.join()
    t_camera.join()
    lidar.stop()
    cv2.destroyAllWindows()
    print("All threads and processes stopped. Exiting.")
