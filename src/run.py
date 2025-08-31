import lib.ros_robot_controller_sdk as rrc
import time
import os

board = rrc.Board()
board.enable_reception()
time.sleep(1)
board.set_rgb([[1, 255, 255, 255], [2, 255, 255, 255]])
board.set_buzzer(1700, 0.1, 0.6, 2)

while True:
  data = board.get_button()
  if data is None:
    time.sleep(0.03)
    continue
  button, event = data
  if button == 1:
    board.set_rgb([[1, 127, 127, 127], [2, 127, 127, 127]])
    board.set_buzzer(1900, 0.1, 0.45, 1)
    os.execlp("python3", "python3", "OpenChallenge.py")
  elif button == 2:
    board.set_rgb([[1, 255, 0, 0], [2, 0, 255, 0]])
    board.set_buzzer(1900, 0.1, 0.3, 2)
    os.execlp("python3", "python3", "ObstacleChallenge.py")
