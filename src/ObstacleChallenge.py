import threading
import multiprocessing
import ctypes
import cv2
import ros_robot_controller_sdk as rrc
import time
from picamera2 import Picamera2
import numpy as np
import math

# --- Image Correction Functions (from V3.1) ---
def compute_gain_map(img_reference: np.ndarray, patch_coords: tuple, blur_kernel_size: int = 201) -> np.ndarray:
    """
    Compute a smoothed per-pixel gain map from a white reference image.
    """
    if blur_kernel_size % 2 == 0:
        blur_kernel_size += 1

    ref = img_reference.astype(np.float32)
    x, y, w, h = patch_coords
    patch = ref[y : y + h, x : x + w]
    target_bgr = cv2.mean(patch)[:3]

    inv_ref = 1.0 / (ref + 1e-8)
    gain_map = np.empty_like(ref, dtype=np.float32)
    for c in range(3):
        gain_map[:, :, c] = target_bgr[c] * inv_ref[:, :, c]
        gain_map[:, :, c] = cv2.GaussianBlur(gain_map[:, :, c], (blur_kernel_size, blur_kernel_size), 0)
    return gain_map


def apply_gain_map(img_target: np.ndarray, gain_map: np.ndarray) -> np.ndarray:
    """
    Apply a precomputed gain map to a target image.
    """
    img_float = img_target.astype(np.float32)
    corrected = img_float * gain_map
    return cv2.convertScaleAbs(corrected)

def filter_gray(img: np.ndarray, threshold: int) -> np.ndarray:
    """
    Converts BGR image to grayscale, sets non-gray-ish pixels to white.

    Args:
        img (np.ndarray): BGR image (HxWx3).
        threshold (int): Max channel difference to be considered gray-ish.

    Returns:
        np.ndarray: Grayscale image with non-gray pixels set to white.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    diff_rg = cv2.absdiff(r, g)
    diff_rb = cv2.absdiff(r, b)
    diff_gb = cv2.absdiff(g, b)
    mask = (diff_rg < threshold) & (diff_rb < threshold) & (diff_gb < threshold)
    gray[~mask] = 255
    return gray

# --- Thread 1: Camera Capture ---
def cameraThread(stopped, gain_map):
    """
    Dedicated thread to continuously capture frames from the camera.
    This acts as the 'producer'.
    """
    global global_frame, frame_lock
    picam2 = Picamera2()
    picam2.configure(
        picam2.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            sensor={"output_size": (1280, 960)},
        )
    )
    picam2.start()

    while not stopped.value:
        frame = apply_gain_map(picam2.capture_array("main"), gain_map)
        with frame_lock:
            global_frame = frame

    picam2.stop()
    print("Camera Thread Stopped")


# --- Thread 2: Wall Following and Motor Control ---
def wallFollowThread(stopped, error_pillar, roi_queue):
    """
    High-frequency thread for steering and primary robot control.
    """
    global global_frame, frame_lock, wallFollow_display

    board = rrc.Board(timeout=1, write_timeout=1)

    # Arm Servos and Motors
    ServoChannel = 4
    MotorChannel = 1
    ServoSpeed = 0.01
    MotorTransitionSpeed = 0.01
    motorPW = 1630
    servoStraight = 1900
    servoPW = servoStraight
    prev_servoPW = servoPW
    last_time = time.time()

    board.pwm_servo_set_position(ServoSpeed, [[ServoChannel, servoStraight]])
    board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, 1500]])
    time.sleep(1)
    board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, motorPW]])

    # PID and State Variables
    stMode = False
    detected = False
    threshold = 150
    Kp, oKp = 0.08, 0.08
    Kd, oKd = 0.10, 0.10
    Ki = 0
    last_error = 0
    i_error = 0

    try:
        while not stopped.value:
            if global_frame is None:
                continue

            cur_time = time.time()
            if cur_time - last_time < (1 / 100):
                continue
            last_time = cur_time

            with frame_lock:
                img = global_frame.copy()
            
            gray_img = filter_gray(img, 40)
            display_img = cv2.cvtColor(gray_img.copy(), cv2.COLOR_GRAY2BGR)

            # --- Wall Following Logic ---
            ROI_left = gray_img[210:230, 0:190]
            ROI_right = gray_img[210:230, 450:640]
            display_ROI_left = display_img[210:230, 0:190]
            display_ROI_right = display_img[210:230, 450:640]

            _, imgThresh_left = cv2.threshold(ROI_left, threshold, 255, cv2.THRESH_BINARY_INV)
            contours_left, _ = cv2.findContours(imgThresh_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            MaxContourLeft = max(contours_left, key=cv2.contourArea) if contours_left else None
            MaxLeftArea = cv2.contourArea(MaxContourLeft) if MaxContourLeft is not None else 0
            if MaxContourLeft is not None:
                cv2.drawContours(display_ROI_left, [MaxContourLeft], 0, (0, 255, 0), 2)

            _, imgThresh_right = cv2.threshold(ROI_right, threshold, 255, cv2.THRESH_BINARY_INV)
            contours_right, _ = cv2.findContours(imgThresh_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            MaxContourRight = max(contours_right, key=cv2.contourArea) if contours_right else None
            MaxRightArea = cv2.contourArea(MaxContourRight) if MaxContourRight is not None else 0
            if MaxContourRight is not None:
                cv2.drawContours(display_ROI_right, [MaxContourRight], 0, (0, 255, 0), 2)

            # --- Send ROI to Obstacle Process ---
            # Extract the ROI needed for the other process and put it in the queue
            roi_for_process = img[front_coords[1] : front_coords[3], front_coords[0] : front_coords[2]]
            if not roi_queue.full():
                roi_queue.put(roi_for_process)

            # --- Steering Calculation ---
            current_error_pillar = error_pillar.value
            error = current_error_pillar if (current_error_pillar != 0) else (MaxRightArea - MaxLeftArea)
            error = error * (math.log(abs(error) + 200) / 5)

            derivative = error - last_error
            last_error = error
            i_error += error

            if abs(error) > 100:
                steering_correction = error * Kp + i_error * Ki + derivative * Kd
                steer = servoStraight + int(steering_correction)
            else:
                steer = servoStraight

            steer = max(1650, min(2250, steer))
            servoPW = steer
            if servoPW != prev_servoPW:
                prev_servoPW = servoPW
                board.pwm_servo_set_position(ServoSpeed, [[ServoChannel, servoPW]])

            if abs(error) > 1500 and not stMode:
                Kp = 0.09
                Kd = 0.09
                stMode = True
                print("Steering Mode ON")
            elif abs(error) < 500 and stMode:
                Kp = oKp
                Kd = oKd
                stMode = False
                print("Steering Mode OFF")

            # --- Update Display Image ---
            cv2.rectangle(display_img, (0, 210), (190, 230), (255, 0, 0), 2)
            cv2.rectangle(display_img, (450, 210), (640, 230), (255, 0, 0), 2)

            cv2.putText(
                display_img,
                f"Error: {error}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 255),
                2,
            )

            with frame_lock:
                wallFollow_display = display_img

    finally:
        # Graceful shutdown of motors
        board.pwm_servo_set_position(MotorTransitionSpeed, [[MotorChannel, 1500]])
        print("WallFollow Thread Stopped")


def filterSolids(contour):
    contourArea = cv2.contourArea(contour)
    convexHullArea = cv2.contourArea(cv2.convexHull(contour))
    x, y, w, h = cv2.boundingRect(contour)
    rectangularity = float(contourArea) / (w * h + 1e-8)
    solidity = float(contourArea) / (convexHullArea + 1e-8)
    if contourArea < 1000:
        return solidity > 0.4 and rectangularity > 0.5
    else:
        return solidity > 0.6 and rectangularity > 0.3
# --- Process 1: Obstacle Detection ---
def obstacleChallengeProcess(stopped, error_pillar, roi_queue, obstacle_display_queue):
    """
    CPU-intensive process for detecting red/green obstacles.
    Receives a small ROI via a queue to minimize data transfer.
    Now also sends back display information.
    """
    lower_red = np.array([50, 160, 130])
    upper_red = np.array([130, 195, 180])
    lower_green = np.array([40, 70, 130])
    upper_green = np.array([210, 140, 200])
    lower_blue = np.array([75, 115, 50])
    upper_blue = np.array([140, 180, 100])
    steerCount = 0
    last_steer = -1
    detected = False

    while not stopped.value:
        try:
            # Block and wait for the ROI from the wall-following thread
            ROI_front = roi_queue.get(timeout=1)
        except Exception:
            # If queue is empty for 1s, check if we should exit
            continue

        # Create display ROI for visualization
        display_ROI_front = ROI_front.copy()
        img_lab = cv2.cvtColor(ROI_front, cv2.COLOR_BGR2LAB)

        # Red Detection
        mask_red = cv2.inRange(img_lab, lower_red, upper_red)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_red = list(filter(filterSolids, contours_red))
        MaxRedArea = 0
        redcenter_x = 0
        if contours_red:
            MaxRedCnt = max(contours_red, key=cv2.contourArea)
            MaxRedArea = cv2.contourArea(MaxRedCnt)
            if MaxRedArea > 200:
                # Draw contours and bounding box for red obstacles
                cv2.drawContours(display_ROI_front, [MaxRedCnt], -1, (0, 0, 255), 2)
                approx = cv2.approxPolyDP(MaxRedCnt, 0.01 * cv2.arcLength(MaxRedCnt, True), True)
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(display_ROI_front, (x, y), (x + w, y + h), (255, 0, 255), 2)
                redcenter_x = x + w / 2

        # Green Detection
        mask_green = cv2.inRange(img_lab, lower_green, upper_green)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green = list(filter(filterSolids, contours_green))
        MaxGreenArea = 0
        greencenter_x = 0
        if contours_green:
            MaxGreenCnt = max(contours_green, key=cv2.contourArea)
            MaxGreenArea = cv2.contourArea(MaxGreenCnt)
            if MaxGreenArea > 200:
                # Draw contours and bounding box for green obstacles
                cv2.drawContours(display_ROI_front, [MaxGreenCnt], -1, (0, 255, 0), 2)
                approx = cv2.approxPolyDP(MaxGreenCnt, 0.01 * cv2.arcLength(MaxGreenCnt, True), True)
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(display_ROI_front, (x, y), (x + w, y + h), (255, 0, 0), 2)
                greencenter_x = x + w / 2
                
        mask_blue = cv2.inRange(img_lab, lower_blue, upper_blue)

        cur_time = time.time()
        if cur_time - last_steer > (1 if detected else 4):
            contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            MaxBlueArea = 0
            if contours_blue:
                MaxBlueCnt = max(contours_blue, key=cv2.contourArea)
                MaxBlueArea = cv2.contourArea(MaxBlueCnt)
                if not detected:
                    if MaxBlueArea > 20 and MaxBlueArea < 150:
                        cv2.drawContours(display_ROI_front, [MaxBlueCnt], -1, (255, 0, 0), 2)
                        detected = True
                        last_steer = cur_time
                else:
                    if MaxBlueArea > 800:
                        cv2.drawContours(display_ROI_front, [MaxBlueCnt], -1, (255, 0, 0), 2)
                        detected = False
                        steerCount += 1
                        last_steer = cur_time
                        print(steerCount, flush=True)

        # Calculate error and send it back to the main thread
        current_error = 0
        roi_width = ROI_front.shape[1]
        if max(MaxRedArea, MaxGreenArea) > 200:
            if MaxRedArea > MaxGreenArea:
                current_error = redcenter_x - (roi_width / 2) + 375
            else:
                current_error = greencenter_x - (roi_width / 2) - 175
            current_error = -current_error
            current_error *= 1.5
            current_error *= min(max(0.3, math.log(max(MaxRedArea, MaxGreenArea) / 3) / 2), 15)
            if abs(current_error) < 50:
                current_error = 0
        error_pillar.value = current_error

        # Send display information back to main thread
        display_data = {
            "roi": display_ROI_front,
            "MaxRedArea": MaxRedArea,
            "MaxGreenArea": MaxGreenArea,
            "redcenter_x": redcenter_x,
            "greencenter_x": greencenter_x,
            "error_pillar": current_error,
            "steerCount": steerCount
        }

        if not obstacle_display_queue.full():
            obstacle_display_queue.put(display_data)
        
        if steerCount == 4 and detected:
            stopped.value = True

    print("ObstacleChallenge Process Stopped")


if __name__ == "__main__":
    # --- Shared Memory and Flags ---
    # For communication between processes
    stopped = multiprocessing.Value("b", False)
    error_pillar = multiprocessing.Value("d", 0.0)
    roi_queue = multiprocessing.Queue(maxsize=1)  # Queue to send ROI
    obstacle_display_queue = multiprocessing.Queue(maxsize=1)  # Queue to receive display data

    # For communication between threads
    global_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame_lock = threading.Lock()
    wallFollow_display = np.zeros((480, 640, 3), dtype=np.uint8)

    print("Initializing...")
    gain_map = compute_gain_map(cv2.imread("Correction_Paper.png"), (85, 66, 543, 361))

    # --- Create and Start Threads and Processes ---
    p_obstacle = multiprocessing.Process(
        target=obstacleChallengeProcess,
        args=(stopped, error_pillar, roi_queue, obstacle_display_queue)
    )
    t_camera = threading.Thread(target=cameraThread, args=(stopped, gain_map))
    t_wallFollow = threading.Thread(
        target=wallFollowThread,
        args=(stopped, error_pillar, roi_queue),
    )

    print("Starting Process and Threads...")
    p_obstacle.start()
    t_camera.start()
    t_wallFollow.start()

    # --- Main Display Loop ---
    last_display_time = 0
    display_interval = 1 / 30  # 30 FPS
    front_coords = (70, 130, 570, 350)

    try:
        while True:
            now = time.time()
            if (now - last_display_time) >= display_interval:
                with frame_lock:
                    # Make copies for safe display
                    wf_image = wallFollow_display.copy()

                    # Create obstacle display image
                    oc_image = global_frame.copy()

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
                            f"Steer Count: {display_data['steerCount']:.1f}",
                            (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 0),
                            2,
                        )

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

                    except:
                        # No obstacle data available, just show the basic image
                        pass

                cv2.imshow("Wall Following", wf_image)
                cv2.imshow("Obstacle Challenge", oc_image)

                last_display_time = now

            if cv2.waitKey(1) == ord("q"):
                print("Stopping...")
                stopped.value = True
                break
    finally:
        # --- Cleanup ---
        p_obstacle.join()
        t_wallFollow.join()
        t_camera.join()
        cv2.destroyAllWindows()
        print("All threads and processes stopped. Exiting.")
        exit(0)
