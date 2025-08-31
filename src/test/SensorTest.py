import cv2
from picamera2 import Picamera2 # pyright: ignore[reportMissingImports]
import queue
import sys
sys.path.append('../')
from lidar import LiDAR
from imu_scan import imu_value
from utils import imshow, waitKey, destroyAllWindows

lidar = LiDAR()
visualizer, lidar_roi_queue = lidar.get_visualizer()

picam2 = Picamera2()

picam2.configure(
picam2.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    sensor={"output_size": (1640, 1232)}
)
)
picam2.start()
lidar.add_roi(lidar_roi_queue, (0, 255, 0), (-1450, -750), (1450, 750))
lidar.add_roi(lidar_roi_queue, (255, 0, 255), (-10, -10), (500, -50))
while True:
    img = picam2.capture_array()
    imshow("Camera", img)
    try:
        visualizer_image = visualizer.get_nowait()
        imshow("Visualizer", visualizer_image)
    except queue.Empty:
        pass
    print(imu_value.value)
    if waitKey(1)==ord('q'):
        break
lidar.stop()
destroyAllWindows()
