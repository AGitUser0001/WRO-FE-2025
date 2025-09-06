import time
from multiprocessing import sharedctypes
from lidar import LiDAR
from types import FunctionType
from utils import sign

class Parking:
  def __init__(self):
    pass
  
  def exit_parking_lot(self, servoStraight: int, parkMotorPW: int, last_error: float, setServo: FunctionType, setMotor: FunctionType):
    n = 4
    direction = sign(last_error)
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
      time.sleep(0.3)
      setServo(servoStraight)
      if direction > 0:
        time.sleep(0.4)
      else:
        time.sleep(0.05)
      if i == n - 1: break
    return direction
  
  def getDist(self, side = 'inner', offset = 0, rtype = 'dist'):
    if side != 'front':
      offset *= -self.direction
    if side == 'outer':
      offset += 180
    if side == 'front':
      return LiDAR.get_angle_median(self.lidar_array, offset + 0 - 3, offset + 0 + 3)[4 if rtype == 'xy' else 1]
    if self.direction == -1:
      return abs(LiDAR.get_angle_median(self.lidar_array, offset + 270 - 3, offset + 270 + 3)[3 if rtype == 'xy' else 1])
    if self.direction == 1:
      return abs(LiDAR.get_angle_median(self.lidar_array, offset + 90 - 3, offset + 90 + 3)[3 if rtype == 'xy' else 1])

  def enter_parking_lot(self, lidar: LiDAR, stopped: sharedctypes.Synchronized, status: sharedctypes.Synchronized,
                        direction: int):
    lidar_array, lidar_shm = lidar.get_array_copy()
    #pylint: disable=W0201
    self.lidar_array = lidar_array
    self.lidar_shm = lidar_shm
    self.direction = direction
    self.stopped = stopped
    self.status = status
    return self.stage1, None
    
  def stage1(self):
    sideDist = self.getDist()
    sideDist2 = self.getDist(offset=25)
    avg_dist = (sideDist + sideDist2) / 2
    targetDist = 300
    error = (targetDist - avg_dist) * self.direction
    return self.stage1, error
  
  def last_stage(self):
    self.lidar_shm.close()
