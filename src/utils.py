import cv2
import time
import numpy as np
import os

def drawContour(contour, image, *, bounding_box=True, c_colour=(0, 0, 0), b_colour=(255, 0, 255)):
  cv2.drawContours(image, [contour], 0, c_colour, 2)
  if bounding_box:
    _approx, (x, y, w, h) = getBoundingBox(contour)
    cv2.rectangle(image, (x, y), (x+w, y+h), b_colour, 1)
    
def processContours(contours, threshold = 0, draw_image=None, *, draw_bounding_box=True, draw=1, c_colour=None, b_colour=None):
  cntList = [(cnt, cv2.contourArea(cnt)) for cnt in contours if cv2.contourArea(cnt) > threshold]

  if len(cntList) < 1: cntList.append((None, 0))
  MaxCnt, MaxCntArea = max(cntList, key=lambda x: x[1])
  approx, bounding_box = None, (0, 0, 0, 0)
  if MaxCnt is not None:
    approx, bounding_box = getBoundingBox(MaxCnt)
  if draw_image is not None and draw != 0 and MaxCnt is not None:
    kwargs = {}
    if c_colour is not None: kwargs["c_colour"] = c_colour
    if b_colour is not None: kwargs["b_colour"] = b_colour
    for cnt, _area in getMaxContours(cntList, draw):
      drawContour(cnt, draw_image, bounding_box=draw_bounding_box, **kwargs)
  return cntList, MaxCnt, MaxCntArea, approx, bounding_box

def getMaxContours(cntList, num = 1):
  if num == 1: return [max(cntList, key=lambda x: x[1])]
  if num < 0: return cntList
  return sorted(cntList, key=lambda x: x[1], reverse=True)[:num]

def getBoundingBox(contour):
  approx = cv2.approxPolyDP(contour, 0.01*cv2.arcLength(contour, True), True)
  bounding_box = cv2.boundingRect(approx)
  return approx, bounding_box

def get_timer(timer_dict: dict, low_timer: float, high_timer: float):
  prev_time = timer_dict.get("time", -1)
  value = timer_dict.get("value", False)
  cur_time = time.time()
  result = cur_time - prev_time > (high_timer if value else low_timer)
  return result, value

def set_timer(timer_dict: dict, new_value: bool | None = None, skip_time = False):
  value = not timer_dict.get("value", False) if new_value is None else new_value
  cur_time = -1 if skip_time else time.time()
  timer_dict["value"] = value
  timer_dict["time"] = cur_time
  return value

def posToDegrees(servoPos):
  return (servoPos if servoPos < 1000 else servoPos - 1800) / 10.2

def getCollisions(mask: np.ndarray, pt1: tuple[int, int], pt2: tuple[int, int], thickness = 1):
  line_img = np.zeros_like(mask)
  cv2.line(line_img, pt1, pt2, 255, thickness)

  collision_pixels = cv2.bitwise_and(line_img, mask)
  return cv2.countNonZero(collision_pixels)

def sign(num):
  return int(np.sign(num))

def imshow(winname, mat):
  if has_display:
    cv2.imshow(winname, mat)

def waitKey(delay):
  if has_display:
    return cv2.waitKey(delay)
  return -1

def destroyAllWindows():
  if has_display:
    cv2.destroyAllWindows()

has_display = os.environ.get("DISPLAY", "NONE") != "NONE"
