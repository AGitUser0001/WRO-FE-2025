import lib.ros_robot_controller_sdk as rrc
import time
import os

MODE = "OpenChallenge"
#MODE = "ObstacleChallenge"

script_dir = os.path.dirname(__file__)
board = rrc.Board()
board.enable_reception()
time.sleep(1)

if MODE == "OpenChallenge":
  board.set_rgb([[1, 255, 255, 255], [2, 255, 255, 255]])
if MODE == "ObstacleChallenge":
  board.set_rgb([[1, 0, 127, 0], [2, 127, 0, 0]])
board.set_buzzer(1700, 0.2, 0.8, 2)
time.sleep(2 * (0.2 + 0.8))
board.set_buzzer(1900, 0.25, 0.8, 1)

while True:
  data = board.get_button()
  if data is None:
    time.sleep(0.03)
    continue
  button, event = data
  if button != 1: continue
  if MODE == "OpenChallenge":
    board.set_rgb([[1, 127, 127, 127], [2, 127, 127, 127]])
    time.sleep(1)
    board.set_buzzer(1900, 0.1, 0.7, 2)
    board.set_rgb([[1, 0, 0, 127], [2, 0, 0, 127]])
    os.execlp("python3", "python3", script_dir + "/OpenChallenge.py")
  elif MODE == "ObstacleChallenge":
    board.set_rgb([[1, 255, 0, 0], [2, 0, 255, 0]])
    time.sleep(1)
    board.set_buzzer(1900, 0.1, 0.7, 2)
    board.set_rgb([[1, 127, 0, 0], [2, 0, 127, 0]])
    os.execlp("python3", "python3", script_dir + "/ObstacleChallenge.py")
