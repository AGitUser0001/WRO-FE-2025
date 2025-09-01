import numpy as np
import math
import cv2
import time
import queue
from utils import getCollisions, processContours, getMaxContours, getBoundingBox, get_timer, set_timer, sign

class ObstacleChallengeProcess():
  lower_red = np.array([35, 150, 124])
  upper_red = np.array([140, 255, 255])
  lower_green = np.array([35, 0, 124])
  upper_green = np.array([190, 107, 255])
  lower_blue = np.array([0, 120, 0])
  upper_blue = np.array([255, 160, 110])
  lower_magenta = np.array([0, 155, 63])
  upper_magenta = np.array([140, 255, 130])
  turn_limit = 12
  def __init__(self, stopped, error_pillar, roi_queue, obstacle_display_queue, status):
    turnCount = 0
    last_turn_detection = -1
    last_parking_detect = -1
    obs_timer = {}
    obs_collision_timer = {}
    detected_turn = False
    parking = False
    parking_side = 0
    parking_detected = 0
    Kp = 0.8
    Kd = 0.5
    last_error = 0
    last_time = -1
    avg_dt = 1/30
    
    while not stopped.value:
      try:
        # Block and wait for the ROI from the wall-following thread
        ROI_front = roi_queue.get(timeout=1)
      except queue.Empty:
        continue
      
      cur_time = time.time()
      dt = cur_time - last_time
      last_time = cur_time
      avg_dt = (avg_dt * 0.95) + (dt * 0.05)

      rw = ROI_front.shape[1]
      rh = ROI_front.shape[0]
      
      # Create display ROI for visualization
      display_ROI_front = ROI_front.copy()
      ROI_front_LAB = cv2.cvtColor(ROI_front, cv2.COLOR_BGR2LAB)
      ROI_front_grey = cv2.cvtColor(ROI_front, cv2.COLOR_BGR2GRAY)

      # Red Detection
      red_mask, _, _, MaxRedArea, _, _, red_x, _, red_y =self.detect_contours(ROI_front_LAB, self.lower_red, self.upper_red, 400, draw_image=display_ROI_front, c_colour=(0, 0, 255), conditional=ROI_front_LAB[:, :, 1] > ROI_front_LAB[:, :, 2])
      
      # Green Detection
      green_mask, _, _, MaxGreenArea, _, _, green_x, _, green_y =self.detect_contours(ROI_front_LAB, self.lower_green, self.upper_green, 400, draw_image=display_ROI_front, c_colour=(0, 255, 0))
      
      obs_timer_res, obs_timer_high = get_timer(obs_timer, 0.15, 0.1)
      if max(MaxGreenArea, MaxRedArea) > 0 and obs_timer_res and not obs_timer_high:
        set_timer(obs_timer, True)
      if obs_timer_res and obs_timer_high:
        set_timer(obs_timer, False, max(MaxGreenArea, MaxRedArea) > 0)
        MaxGreenArea = 0
        MaxRedArea = 0
      elif not obs_timer_res and not obs_timer_high:
        MaxGreenArea = 0
        MaxRedArea = 0

      _, _, _, MaxBlueArea, _, _, _, _, _ =self.detect_contours(ROI_front_LAB, self.lower_blue, self.upper_blue, draw_image=display_ROI_front)
      # Blue Line Detection
      cur_time = time.time()
      if turnCount < self.turn_limit and cur_time - last_turn_detection > (0.2 if detected_turn else 3):
        if MaxBlueArea != 0:
          if not detected_turn:
            if MaxBlueArea > 300 and MaxBlueArea < 900:
              detected_turn = True
              last_turn_detection = cur_time
          else:
            if MaxBlueArea > 700:
              detected_turn = False
              turnCount += 1
              last_turn_detection = cur_time
      
      # Calculate error and send it back to the main thread
      current_error = 0
      distance = 0
      offset = 0
      red_x_relative = red_x - (rw / 2)
      green_x_relative = green_x - (rw / 2)
      obs_y = rh - 1
      robot_pos_absolute = (int(rw / 2), 480 - 140)
      robot_pos_relative = (0, robot_pos_absolute[1])
      if MaxRedArea > 0 or MaxGreenArea > 0:
        x_relative = 0
        if MaxRedArea > MaxGreenArea:
          x_relative = red_x_relative
          obs_y = red_y
          offset = 1
        else:
          x_relative = green_x_relative
          obs_y = green_y
          offset = -1

        distance = math.dist(robot_pos_relative, (x_relative, obs_y))
        K_max = 2.5
        d_min = 120               # math.dist((0, 480 - 140), (0, 220))
        d_max = 422.0189569201839 # math.dist((0, 480 - 140), (250, 0))

        K_obs = K_max * (d_max - distance) / (d_max - d_min)
        K_obs = max(0, min(K_max, K_obs))

        offset *= 750
        offset *= (K_obs / K_max) ** 2.1

        current_error = x_relative * K_obs
        current_error += offset

      derivative = current_error - last_error
      last_error = current_error
      
      current_error = Kp * current_error + Kd * derivative if abs(current_error) > 0 else 0
      target_pos = (int(current_error + rw / 2), obs_y)
      num_collisions = getCollisions(cv2.threshold(ROI_front_grey, 60, 255, cv2.THRESH_BINARY_INV)[1], robot_pos_absolute, target_pos, 2)
      if num_collisions > 35:
        current_error = 0
      
      if current_error != 0:
        cv2.line(display_ROI_front, robot_pos_absolute, target_pos, (255, 0, 0), thickness=2)
        cv2.circle(display_ROI_front, target_pos, radius=3, color=(255, 0, 0), thickness=-1)

      obs_detect_line = (robot_pos_absolute, (int(rw / 2), rh - 100))
      num_obs_collisions = getCollisions(cv2.bitwise_or(red_mask, green_mask), *obs_detect_line)
      will_collide_with_obs = num_obs_collisions > 20
      obs_collision_timer_res, obs_collision_timer_high = get_timer(obs_collision_timer, 1, 0.2)
      if will_collide_with_obs and obs_collision_timer_res and not obs_collision_timer_high:
        set_timer(obs_collision_timer, True)
      if obs_collision_timer_res and obs_collision_timer_high:
        set_timer(obs_collision_timer, False, will_collide_with_obs)
      if not obs_collision_timer_res and not obs_collision_timer_high:
        status.value = b"BACKWARD"
        cv2.line(display_ROI_front, *obs_detect_line, (100, 100, 100), thickness=1)
      else:
        status.value = b"FORWARD"
        cv2.line(display_ROI_front, *obs_detect_line, (200, 200, 200) if will_collide_with_obs else (255, 255, 255), thickness=1)

      if parking:
        parking_detected, current_error, parking_side, last_parking_detect = self.parking(ROI_front_LAB, display_ROI_front, rw, rh, status, error_pillar, stopped, parking_detected, parking_side, last_parking_detect)
        
      error_pillar.value = current_error
      # Send display information back to main thread
      display_data = {
        "roi": display_ROI_front,
        "MaxRedArea": MaxRedArea,
        "MaxGreenArea": MaxGreenArea,
        "redcenter_x": red_x,
        "greencenter_x": green_x,
        "error_pillar": current_error,
        "steerCount": turnCount,
        "distance": distance,
        "offset": offset,
        "parking_detected": parking_detected,
        "parking": parking,
        "avg_dt": avg_dt
      }

      if not obstacle_display_queue.full():
        obstacle_display_queue.put(display_data)
      
      if turnCount == self.turn_limit:
        #parking = True
        stopped.value = True

    roi_queue.cancel_join_thread()
    obstacle_display_queue.cancel_join_thread()
    print("ObstacleChallenge Process Stopped")

  def filterSolids(self, contour):
    contourArea = cv2.contourArea(contour)
    convexHullArea = cv2.contourArea(cv2.convexHull(contour))
    _x, _y, w, h = cv2.boundingRect(contour)
    rectangularity = float(contourArea) / (w * h + 1e-8)
    solidity = float(contourArea) / (convexHullArea + 1e-8)
    if contourArea < 1000:
      return solidity > 0.5 and rectangularity > 0.2
    elif contourArea < 2000:
      return solidity > 0.4 and rectangularity > 0.3
    else:
      return True

  def detect_contours(self, img_lab, lower_lab, upper_lab, threshold = 200, draw_image = None, *, conditional=None, filterSolids=True, draw_bounding_box=True, draw=1, c_colour=None, b_colour=None):
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
    return mask, cntList, MaxCnt, MaxCntArea, approx, bounding_box, center_x, center_y, bottom_y
  
  def parking(self, ROI_front_LAB, display_ROI_front, rw, rh, status, error_pillar, stopped, parking_detected, parking_side, last_parking_detect):
    limit = 500
    magenta1center_x = rw / 2
    magenta1center_y = rh / 2
    magenta1bottom_y = rh / 2
    magenta2center_x = rw / 2
    magenta2center_y = rh / 2
    magenta2bottom_y = rh / 2
    current_error = 0
    
    _, cntList, _, _, _, _, _, _, _ =self.detect_contours(ROI_front_LAB, self.lower_magenta, self.upper_magenta, limit, draw_image=display_ROI_front, draw=2)
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

    wall1 = MaxMagentaArea1 > 0
    wall2 = MaxMagentaArea2 > 0
    avg_x = (magenta1center_x + magenta2center_x) // 2 if wall2 else magenta1center_x
    avg_y = (magenta1center_y + magenta2center_y) // 2 if wall2 else magenta1center_y
    avg_y_bottom = (magenta1bottom_y + magenta2bottom_y) // 2 if wall2 else magenta1bottom_y
    avg_x_diff = avg_x - rw / 2
    cv2.circle(display_ROI_front, (int(avg_x), int(avg_y)), radius=3, color=(0, 255, 255), thickness=-1)
    cv2.circle(display_ROI_front, (int(avg_x), int(avg_y_bottom)), radius=3, color=(255, 0, 255), thickness=-1)
    
    distance = (1 / math.log((MaxMagentaArea1 + MaxMagentaArea2) / 1.4)) * 10 if MaxMagentaArea1 > 0 else 0
    
    parking_tracker = avg_x_diff - (sign(avg_x_diff) * max(-10 / distance, abs(avg_x_diff))) if avg_x_diff != 0 else 0

    cur_time = time.time()
    if parking_detected == 0 and wall1:
      last_parking_detect = cur_time
      parking_detected = 0.5
      
    elif parking_detected == 0.5 and cur_time - last_parking_detect > 0.5:
      if wall1:
        parking_detected += 0.5
      else:
        parking_detected -= 0.5

    elif parking_detected == 1 or parking_detected == 1.5:
      current_error = parking_tracker * -10

      if parking_detected == 1 and wall1 and wall2:
        last_parking_detect = cur_time
        parking_detected = 1.5   
      
      if parking_detected == 1.5 and cur_time - last_parking_detect > 0.1:
        if wall1 and wall2:
          parking_detected += 0.5
          parking_side = sign(avg_x_diff)
        else:
          parking_detected -= 0.5
    
    elif parking_detected == 2 or parking_detected == 2.5:
      current_error = parking_tracker * -10
      
      if parking_detected == 2 and wall1 and not wall2:
        last_parking_detect = cur_time
        parking_detected = 2.5
        status.value = b"FORWARD_SLOW"
    
      if parking_detected == 2.5 and cur_time - last_parking_detect > 0.2:
        if wall1 and not wall2:
          parking_detected += 0.5
        else:
          parking_detected -= 0.5
          
    elif parking_detected == 3 or parking_detected == 3.5:
      status.value = b"FORWARD_SLOW"
      current_error = parking_tracker * -10
      if parking_detected == 3 and not wall1 and not wall2:
          parking_detected += 0.5
          
      if parking_detected == 3.5 and cur_time - last_parking_detect > 0.1:
        if not wall1 and not wall2:
          parking_detected += 0.5
        else:
          parking_detected -= 0.5
        
    elif parking_detected == 4:
      status.value = b"BACKWARD"
      error_pillar.value = 1 * parking_side
      time.sleep(0.1)
      error_pillar.value = -10000 * parking_side
      time.sleep(1.2)
      error_pillar.value = 1 * parking_side
      time.sleep(1.1)
      for _i in range(2):
        error_pillar.value = 10000 * parking_side
        time.sleep(0.45)
        status.value = b"FORWARD"
        error_pillar.value = -10000 * parking_side
        time.sleep(0.25)
        status.value = b"BACKWARD"
        error_pillar.value = 10000 * parking_side
        time.sleep(0.45)
        status.value = b"FORWARD"
        error_pillar.value = -10000 * parking_side
        time.sleep(0.2)
        status.value = b"BACKWARD"
      error_pillar.value = 10000 * parking_side
      time.sleep(0.45)
      status.value = b"FORWARD"
      error_pillar.value = -10000 * parking_side
      time.sleep(0.2)
      status.value = b"BACKWARD"
      error_pillar.value = 10000 * parking_side
      time.sleep(0.5)
      for _i in range(2):
        status.value = b"FORWARD"
        error_pillar.value = -10000 * parking_side
        time.sleep(0.2)
        status.value = b"BACKWARD"
        error_pillar.value = 10000 * parking_side
        time.sleep(0.45)
      stopped.value = True
    return parking_detected, current_error, parking_side, last_parking_detect
