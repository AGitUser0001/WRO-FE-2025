import time
from multiprocessing import sharedctypes
from lidar import LiDAR
from types import FunctionType
from utils import sign
from imu_scan import imu_value

class Parking:
  def exit_parking_lot(self, lidar: LiDAR, servoStraight: int, parkMotorPW: int, last_error: float,
                       setServo: FunctionType, setMotor: FunctionType):
    lidar_array, lidar_shm = lidar.get_array_copy()
    #pylint: disable=W0201
    self.lidar_array = lidar_array
    self.lidar_shm = lidar_shm

    max_cycles = 50
    leftDist = 0
    rightDist = 0
    while True:
      leftDist = self.getLiDARValue(90, 3, 'xy')
      rightDist = self.getLiDARValue(270, 3, 'xy')
      print(leftDist, rightDist, imu_value.value)
      if leftDist > 0 and rightDist > 0 and leftDist < 2000 and rightDist < 2000 and imu_value.value != 0:
        break
      time.sleep(0.01)
    last_error = rightDist - leftDist
    direction = sign(last_error)
    self.direction = direction
    OUT = servoStraight + (400 * direction)
    IN = servoStraight - (400 * direction)
    setServo(servoStraight)
    setMotor(1500)
    time.sleep(0.15)
    for i in range(max_cycles):
      frontDist = self.getLiDARValue(0, 3, 'xy')
      yaw = (imu_value.value + 180) % 360 - 180
      abs_yaw = abs(yaw)
      print(yaw, abs_yaw)
      if 59 < abs_yaw <= 67 and (frontDist == 0 or frontDist > 100):
        break

      if abs_yaw < 63:
        self.turnInPlace(servoStraight, OUT, IN, parkMotorPW, setServo, setMotor, 0.5)
      else:
        self.turnInPlace(servoStraight, IN, OUT, parkMotorPW, setServo, setMotor, 0.5)
    
    setServo(servoStraight)
    setMotor(parkMotorPW)
    time.sleep(0.8)
    for i in range(max_cycles):
      yaw = (imu_value.value + 180) % 360 - 180
      print(yaw)
      if -4 <= yaw <= 4:
        break

      if yaw > 0 or yaw < -160:
        self.turnInPlace(servoStraight, IN, OUT, parkMotorPW, setServo, setMotor, 0.5)
      else:
        self.turnInPlace(servoStraight, OUT, IN, parkMotorPW, setServo, setMotor, 0.5)
    setMotor(1500 + (1500 - parkMotorPW))
    time.sleep(0.4)

    setServo(servoStraight)
    setMotor(1500)
    return direction
  
  def turnInPlace(self, FORWARD, OUT, IN, motorPW, setServo, setMotor, delay):
    setServo(OUT)
    setMotor(motorPW)
    time.sleep(delay / 2)
    setServo(IN)
    setMotor(1500 + (1500 - motorPW))
    time.sleep(delay / 2)
    setServo(FORWARD)
    setMotor(1500)

  def getLiDARValue(self, angle, near, rtype = 'dist', absolute = True):
    i = 0
    if rtype == 'xy':
      i = 4 if round(angle / 90) % 2 == 0 else 3
    elif rtype == 'dist':
      i = 1
    ret = LiDAR.get_angle_median(self.lidar_array, angle - near, angle + near)[i]
    return abs(ret) if absolute else ret
  
  def getDist(self, side = 'inner', offset = 0, rtype = 'dist', absolute = True):
    if side != 'front':
      offset *= -self.direction
    if side == 'outer':
      offset += 180
    if side == 'front':
      return self.getLiDARValue(offset, 3, rtype, absolute)
    if self.direction == -1:
      return self.getLiDARValue(270 + offset, 3, rtype, absolute)
    if self.direction == 1:
      return self.getLiDARValue(90 + offset, 3, rtype, absolute)

  def enter_parking_lot(self, stopped: sharedctypes.Synchronized, status: sharedctypes.Synchronized, direction: int):
    #pylint: disable=W0201
    self.stopped = stopped
    self.status = status
    self.direction = direction
    return self.stage1, None
    
  def stage1(self):
    sideDist = self.getDist()
    sideDist2 = self.getDist(offset=25)
    avg_dist = (sideDist + sideDist2) / 2
    targetDist = 300
    error = (targetDist - avg_dist) * self.direction
    return self.stage1, error
  
  def last_stage(self):
    if hasattr(self, 'lidar_shm'):
      self.lidar_shm.close()
      print('Parking Stopped')
