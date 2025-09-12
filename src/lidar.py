import serial
import struct
import threading
import multiprocessing
import numpy as np
from multiprocessing import shared_memory
import queue
import time
import cv2

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
  
  def stop(self, wait=True):
    self.stopped.value = True
    if wait:
      self.process.join()
      if self.__visualizer_thread is not None:
        self.__visualizer_thread.join()
    self.shared_memory.close()

  @classmethod
  def get_angle_median(cls, lidar_array: np.ndarray, angle_low, angle_high):
    return cls.median_filter(cls.get_values(lidar_array, angle_low, angle_high))

  @classmethod
  def get_roi_median(cls, lidar_array: np.ndarray, pt1: tuple[int, int], pt2: tuple[int, int], min_points = 0):
    roi_values = cls.get_roi_values(lidar_array, pt1, pt2)
    if len(roi_values) < min_points:
      return np.zeros(NUM_VALUES, DATA_TYPE)
    return cls.median_filter(roi_values)
  
  @classmethod
  def get_value(cls, lidar_array: np.ndarray, angle):
    return lidar_array[int((angle * DATA_RES) % NUM_POINTS)]
  
  @classmethod
  def get_values(cls, lidar_array: np.ndarray, angle_low, angle_high):
    low = int((angle_low * DATA_RES) % NUM_POINTS)
    high = int((angle_high * DATA_RES) % NUM_POINTS + 1)
    if low <= high:
      values = lidar_array[low:high]
    else:
      values = np.concatenate((lidar_array[low:], lidar_array[:high]))
    return values
  
  @classmethod
  def get_roi_values(cls, lidar_array: np.ndarray, pt1: tuple[int, int], pt2: tuple[int, int], filter_invalid = True):
    x_min, x_max = min(pt1[0], pt2[0]), max(pt1[0], pt2[0])
    y_min, y_max = min(pt1[1], pt2[1]), max(pt1[1], pt2[1])

    x = lidar_array[:, 3]
    y = lidar_array[:, 4]

    roi_mask = (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max)
    if filter_invalid:
      valid_mask = lidar_array[:, 1] != 0
      return lidar_array[roi_mask & valid_mask]
    return lidar_array[roi_mask]
  
  @classmethod
  def median_filter(cls, values: np.ndarray, q1_percent = 25, q3_percent = 80):
    if values.size == 0:
      return np.zeros(NUM_VALUES, DATA_TYPE)
    filtered_values = values[values[:, 1] != 0]
    filtered_values = filtered_values if filtered_values.size != 0 else values
    values = filtered_values

    dists = values[:, 1]

    q1 = np.percentile(dists, q1_percent)
    q3 = np.percentile(dists, q3_percent)
    iqr = q3 - q1

    lower_bound = q1 - 1.5 * iqr
    upper_bound = q3 + 1.5 * iqr

    mask = (dists >= lower_bound) & (dists <= upper_bound)
    masked_values = values[mask]
    masked_values = masked_values if masked_values.size != 0 else filtered_values
    values = masked_values

    return np.median(values, axis=0)

  @classmethod
  def add_roi(cls, lidar_roi_queue: multiprocessing.Queue, colour: tuple[int, int, int], pt1: tuple[int, int], pt2: tuple[int, int]):
    try:
      lidar_roi_queue.put_nowait((colour, pt1, pt2))
      return True
    except queue.Full:
      return False

  def get_array_copy(self):
    shared_array_mem = shared_memory.SharedMemory(name=self.shm_name)
    return np.ndarray((NUM_POINTS, NUM_VALUES), dtype=DATA_TYPE, buffer=shared_array_mem.buf), shared_array_mem
  
  def get_visualizer(self):
    if self.__visualizer_display_queue is None:
      self.__visualizer_display_queue = queue.Queue(1)
      self.__visualizer_lidar_roi_queue = multiprocessing.Queue(maxsize=50)
      self.__visualizer_thread = threading.Thread(target=self.__visualizer, args=(self.__visualizer_display_queue, self.__visualizer_lidar_roi_queue), daemon=True)
      self.__visualizer_thread.start()
    return self.__visualizer_display_queue, self.__visualizer_lidar_roi_queue

  def __visualizer(self, display_queue: queue.Queue, lidar_roi_queue: multiprocessing.Queue):
    HEIGHT = 500
    WIDTH = 500
    PANEL_WIDTH = 200
    RESOLUTION = 6 # mm per pixel
    morph_kernel = np.ones((3, 3), np.uint8)
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

    lidar_array, lidar_shm = self.get_array_copy()
    image = np.zeros((HEIGHT, WIDTH + PANEL_WIDTH, 3), dtype=np.uint8)
    display = image[:, :WIDTH]
    panel = image[:, WIDTH:WIDTH + PANEL_WIDTH]
    draw_template = np.zeros_like(display)

    last_time = -1
    rate_limit = 1/20
    while not self.stopped.value:
      display[:] = 0
      panel[:] = 63
      cv2.line(image, (WIDTH, 0), (WIDTH, HEIGHT), (127, 127, 127), 2)
      
      while not lidar_roi_queue.empty():
        try:
          colour, pt1, pt2 = lidar_roi_queue.get_nowait()
          
          px1 = (int(pt1[0] // RESOLUTION + WIDTH // 2), int(-pt1[1] // RESOLUTION + HEIGHT // 2))
          px2 = (int(pt2[0] // RESOLUTION + WIDTH // 2), int(-pt2[1] // RESOLUTION + HEIGHT // 2))
          px1 = (min(WIDTH - 1, max(0, px1[0])), min(HEIGHT - 1, max(0, px1[1])))
          px2 = (min(WIDTH - 1, max(0, px2[0])), min(HEIGHT - 1, max(0, px2[1])))

          cv2.rectangle(draw_template, px1, px2, colour, 1)
        except queue.Empty:
          break

      cur_time = time.time()
      if cur_time - last_time < rate_limit:
          time.sleep((last_time + rate_limit) - cur_time)
      cur_time = time.time()
      last_time = cur_time

      with self.lock:
        lidar_array_copy = lidar_array.copy()

      # Extract x, y
      x = lidar_array_copy[:, 3] // RESOLUTION + WIDTH // 2
      y = -lidar_array_copy[:, 4] // RESOLUTION + HEIGHT // 2

      # Clip valid points
      mask = (x >= 0) & (x < WIDTH) & (y >= 0) & (y < HEIGHT)
      x = x[mask].astype(np.int32)
      y = y[mask].astype(np.int32)

      display[y, x] = (255, 255, 255)

      display[:] = cv2.morphologyEx(cv2.dilate(display, dilate_kernel, iterations=1), cv2.MORPH_CLOSE, morph_kernel, iterations=1)
      display[draw_template != 0] = draw_template[draw_template != 0]

      cv2.putText(panel, f"Front: {self.get_angle_median(lidar_array_copy, 0 - 4, 0 + 4)[4]:.2f}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
      cv2.putText(panel, f"Right: {self.get_angle_median(lidar_array_copy, 90 - 4, 90 + 4)[3]:.2f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
      cv2.putText(panel, f"Back: {self.get_angle_median(lidar_array_copy, 180 - 4, 180 + 4)[4]:.2f}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
      cv2.putText(panel, f"Left: {self.get_angle_median(lidar_array_copy, 270 - 4, 270 + 4)[3]:.2f}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

      if not display_queue.full():
        display_queue.put_nowait(image.copy())
    lidar_shm.close()
    lidar_roi_queue.cancel_join_thread()
    print("LiDAR Visualizer Stopped")

MIN_DIST = 50
MAX_DIST = 4200
MIN_CONF = 60
AGE_THRESHOLD = 200
AGE_THRESHOLD_NEAR = int(4 * DATA_RES)
INDEX_DISTS = np.minimum(
  np.abs(np.arange(NUM_POINTS)[:, np.newaxis] - np.arange(NUM_POINTS)),
  NUM_POINTS - np.abs(np.arange(NUM_POINTS)[:, np.newaxis] - np.arange(NUM_POINTS))
)

class LiDARProcess:
  def __init__(self, port, stopped, shm_name, lock,):
    # Serial setup
    self.serial = serial.Serial(port, BAUD, timeout=0.1)
    self.buffer = bytearray()

    self.shared_memory = shared_memory.SharedMemory(name=shm_name)
    self.lidar_array = np.ndarray((NUM_POINTS, NUM_VALUES), dtype=DATA_TYPE, buffer=self.shared_memory.buf)

    self.lock = lock

    while not stopped.value:
      self.read_lidar()
    self.shared_memory.close()
    self.shared_memory.unlink()
    print("LiDAR Process Stopped", flush=True)

  def find_packet_start(self, buffer):
    for i in range(len(buffer) - 1):
      if buffer[i] == 0x54 and (buffer[i + 1] & 0xFF) == 0x2C:
        return i
    return -1

  def parse_packet(self, packet):
    if len(packet) != PACKET_LEN:
      return None
    
    speed, start_angle = struct.unpack_from("<HH", packet, 2)
    end_angle, timestamp, crc = struct.unpack_from("<HHB", packet, PACKET_LEN - 5)

    # Scale values
    speed /= 64.0
    start_angle /= 100.0
    end_angle /= 100.0

    # Extract 12 measurements (36 bytes starting at byte 6)
    block = packet[6:6 + 12 * 3]

    # Distances: 12 uint16
    dists = np.frombuffer(block, dtype='<u2', count=12)

    # Confidence: last byte of every 3-byte group
    confs = np.frombuffer(block, dtype=np.uint8).reshape(12, 3)[:, 2]

    return { "speed": speed, "start_angle": start_angle, "end_angle": end_angle,
            "timestamp": timestamp, "crc": crc, "dists": dists, "confs": confs }

  def interpolate_angles(self, start, end, count):
    angle_range = (end - start) % 360
    return (start + np.arange(count) * (angle_range / (count - 1))) % 360

  def read_lidar(self):
    data = None
    try:
      data = self.serial.read(256)
    except serial.SerialException:
      pass
    if data:
      self.buffer += data
      while True:
        idx = self.find_packet_start(self.buffer)
        if idx == -1 or len(self.buffer) - idx < PACKET_LEN:
          break
        packet = self.buffer[idx : idx + PACKET_LEN]
        self.buffer = self.buffer[idx + PACKET_LEN :]
        parsed = self.parse_packet(packet)
        if parsed:
          timestamp = parsed["timestamp"]
          angles = self.interpolate_angles(parsed["start_angle"], parsed["end_angle"], 12)
          angles = ((360 - angles) % 360 + 90) % 360

          dists = parsed["dists"]
          confs = parsed["confs"]

          mask = (dists >= MIN_DIST) & (dists <= MAX_DIST) & (confs >= MIN_CONF)
          dists[~mask] = 0

          rads = np.deg2rad(angles)
          x = dists * np.sin(rads)
          y = dists * np.cos(rads)

          indices = (angles * DATA_RES).astype(np.int32)

          timestamps = np.full_like(dists, timestamp, dtype=np.float32)

          data_to_write = np.column_stack((angles, dists, confs, x, y, timestamps))

          ages_ms = timestamp - self.lidar_array[:, 5]
          old_points = ages_ms > AGE_THRESHOLD
          distances_to_packets = INDEX_DISTS[indices]
          packets_near = np.any(distances_to_packets <= AGE_THRESHOLD_NEAR, axis=0)
          old_mask = packets_near & old_points

          with self.lock:
            self.lidar_array[indices] = data_to_write
            self.lidar_array[old_mask, 1] = 0
