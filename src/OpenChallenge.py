import cv2
import ros_robot_controller_sdk as rrc
import time
from picamera2 import Picamera2
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
Kd = 0.1
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
motorPW = 1630
servoStraight = 1900
servoPW = servoStraight
steer = servoStraight
#Arm the Servo
board.pwm_servo_set_position(ServoSpeed, [[ServoChannel, servoPW]])
time.sleep(1)


board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPW]])                                  
while True:
    img = picam2.capture_array()
    ROI_left = img[200:260, 0:190]
    ROI_right = img[200:260, 450:640]



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
    error = MaxRightArea - MaxLeftArea
    #print("Area of right wall is:", MaxRightArea)
    #print("Area of left wall is:", MaxLeftArea)

    derivative = error - last_error
    last_error = error
    if abs(error) > 100:
        steering_correction = Kp * error + Kd * derivative
        steer = servoStraight + int(steering_correction)

    if steer > 2200:
        steer = 2200
    if steer < 1600:
        steer = 1600
    servoPW = steer
    board.pwm_servo_set_position(ServoSpeed, [[ServoChannel, servoPW]])
    
    #print("servoPW =", servoPW)
    time.sleep(0.03) 
    if abs(error) > 3000 and  not stMode:
        Kp = 0.05
        Kd = 0.01
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
    if cv2.waitKey(1)==ord('q') or steerCount == 18:
        motorPW = 1500
        board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPW]])
        break    
    board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPW]])
         
cv2.destroyAllWindows()

