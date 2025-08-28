import cv2

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
