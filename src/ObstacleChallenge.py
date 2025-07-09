import cv2
import ros_robot_controller_sdk as rrc
import time
from picamera2 import Picamera2
import numpy as np
ServoChannel = 4
MotorChannel = 1
ServoSpeed = 0.01
MotorTransitionSpeed = 0.01
MaxRightArea = 0
MaxLeftArea = 0
leftAreaList = []
rightAreaList = []
steerCount = 0
Kp = 0.03
Kd = 0.06
Ki = 0
i_error = 0
stMode = False
detected = False
oKp = Kp
oKd = Kd
error = 0
last_error = 0
threshold = 40
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.sensor.output_size = (1280, 960)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
board = rrc.Board()
motorPW = 1625
servoStraight = 1900
servoPW = servoStraight
steer = servoStraight


greenAreaList = []
MaxGreenArea = 0
redAreaList = []
MaxRedArea = 0
lower_green = np.array([60, 40, 0])
upper_green = np.array([90, 255, 255])
lower_red = np.array([0, 50, 50])
upper_red = np.array([0, 255, 255])
lower_red1 = np.array([170, 50, 50])
upper_red1 = np.array([190, 255, 255])

#Arm the Servo
board.pwm_servo_set_position(ServoSpeed, [[ServoChannel, servoPW]])
time.sleep(1)


board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPW]])                                  
while True:
    img = picam2.capture_array()
    
    img_float = img.astype(np.float32) / 255.0
    correction_matrix = np.array([[1.8635, 0.0, 0.0],
                                  [0.0, 1.6504, 0.0],
                                  [0.0, 0.0, 1.2525]])
    corrected = np.clip(np.dot(img_float, correction_matrix.T), 0, 1)
    corrected = (corrected * 255).astype(np.uint8)
    img = corrected
    ROI_left = img[200:260, 0:190]
    ROI_right = img[200:260, 450:640]


    
    ROI_front = img[100:330, 110:500]
    
    img_hsv1 = cv2.cvtColor(ROI_front, cv2.COLOR_BGR2HSV)
    mask0 = cv2.inRange(img_hsv1, lower_red, upper_red)
    mask1 = cv2.inRange(img_hsv1, lower_red1, upper_red1)
    
    raw_mask = mask0 | mask1
    
    contours2 = cv2.findContours(raw_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    MaxRedArea = 0
    MaxRedCnt = None
    MaxRI = "idk whatever"
    redcenter_x = 0
    redcenter_y = 0
    for i in range(len(contours2)):
        cnt = contours2[i]
        area = cv2.contourArea(cnt)
        redAreaList.append((area, cnt))
        if area > MaxRedArea:
            MaxRedArea = area
            MaxRedCnt = cnt
            MaxRI = i
    print("The max red area is:", MaxRedArea)
    if(MaxRedArea > 700):

            
        approx = cv2.approxPolyDP(MaxRedCnt, 0.01*cv2.arcLength(MaxRedCnt, True), True)
        x, y, w, h = cv2.boundingRect(approx)
        cv2.rectangle(ROI_front,(x,y), (x+w, y+h), (255,0,255),2)
        redcenter_x = x + w/2
        redcenter_y = y + h/2
        cv2.drawContours(ROI_front, contours2, MaxRI, (0, 0, 255), 2)
        
        
    img_hsv = cv2.cvtColor(ROI_front, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(img_hsv, lower_green, upper_green)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    MaxGreenArea = 0
    MaxGreenCnt = None
    MaxGI = "idk whatever bruh"
    greencenter_x = 0
    greencenter_y = 0
    
    
    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        greenAreaList.append((area, cnt))
        if area > MaxGreenArea:
            MaxGreenArea = area
            MaxGreenCnt = cnt
            MaxGI = i
    print("The max green area is:", MaxGreenArea)
    if(MaxGreenArea > 700):
        cv2.drawContours(ROI_front, contours, MaxGI, (0, 255, 0), 2)
            
        approx = cv2.approxPolyDP(MaxGreenCnt, 0.01*cv2.arcLength(MaxGreenCnt, True), True)
        x, y, w, h = cv2.boundingRect(approx)
        cv2.rectangle(ROI_front,(x,y), (x+w, y+h), (255,0,0),2)
        greencenter_x = x + w/2
        greencenter_y = y + h/2
            
    
    print("Red pillar position is:", redcenter_x, redcenter_y)
    print("Green pillar position is:", greencenter_x, greencenter_y)
    cv2.rectangle(img, (80, 160), (560, 340), (255, 0, 0), 5)
    greenAreaList = []
    redAreaList = []
    error_pillar =  0;
    if (max(MaxRedArea, MaxGreenArea) > 500):
        if (MaxRedArea > MaxGreenArea):
            error_pillar = redcenter_x - 320 - 160
            
        else:
            error_pillar = greencenter_x - 320 + 160
    cv2.imshow("Camera", img)
    LeftimgGray = cv2.cvtColor(ROI_left, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("gray", LeftimgGray)
    ret, imgThresh = cv2.threshold(LeftimgGray, threshold, 255, cv2.THRESH_BINARY_INV)
    #cv2.imshow("threshold", imgThresh)
    contours, hierarchy = cv2.findContours(imgThresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        leftAreaList.append(area)
        MaxLeftArea = max(leftAreaList)
 
       
        if(area > 100):
            cv2.drawContours(ROI_left, contours, i, (0, 255, 0), 2)
            
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(ROI_left,(x,y), (x+w, y+h), (0,0,255),2)
    imgGray = cv2.cvtColor(ROI_right, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("gray", imgGray)
    ret, imgThresh = cv2.threshold(imgGray, threshold, 255, cv2.THRESH_BINARY_INV)
    #cv2.imshow("threshold", imgThresh)
    contours, hierarchy = cv2.findContours(imgThresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        rightAreaList.append(area)
        MaxRightArea = max(rightAreaList)
        
        if(area > 100):
            cv2.drawContours(ROI_right, contours, i, (0, 255, 0), 2)
        
            approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(ROI_right,(x,y), (x+w, y+h), (0,0,255),2)
        cv2.rectangle(img, (0, 260), (190, 200), (255, 0, 0), 5)
        cv2.rectangle(img, (450, 260), (640, 200), (255, 0, 0), 5)
        cv2.imshow("Camera2", img)
    leftAreaList = []
    rightAreaList = []
    error = error_pillar * 8 if abs(error_pillar) > 10 else MaxRightArea - MaxLeftArea
    #print("Area of right wall is:", MaxRightArea)
    #print("Area of left wall is:", MaxLeftArea)

    derivative = error - last_error
    last_error = error
    i_error += error
    if abs(error) > 100:
        steering_correction = error * Kp + i_error * Ki + derivative * Kd;
        steer = servoStraight + int(steering_correction)
    print("i_error", i_error, "integral", i_error * Ki)
    if steer > 2200:
        steer = 2200
    if steer < 1600:
        steer = 1600
    servoPW = steer
    board.pwm_servo_set_position(ServoSpeed, [[ServoChannel, servoPW]])
    
    #print("servoPW =", servoPW)
    time.sleep(0.03) 
    if abs(error) > 3000 and  not stMode:
        Kp = 0.09
        Kd = 0.05
        stMode = True
        print("Steering Mode ON")
    elif abs(error) < 600 and stMode:
        Kp = oKp
        Kd = oKd
        stMode = False 
        print("Steering Mode OFF")
    print("error:", error)
    if (MaxRightArea < 2000 or MaxLeftArea < 2000) and  not detected:
        detected = True
        print("Detected TURN")
    elif abs(error) < 60 and detected:
        detected = False 
        print("Detect COMPLETE")
        steerCount += 1
    if cv2.waitKey(1)==ord('q') or steerCount == 1800:
        motorPW = 1500
        board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPW]])
        break    
    board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPW]])
         
cv2.destroyAllWindows()


