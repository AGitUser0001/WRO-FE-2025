#!/usr/bin/env python3
import subprocess
import threading
import queue

button_queue = queue.Queue(1)

def listen_to_button_events():
  command = 'source /home/ubuntu/.zshrc && ros2 topic echo /ros_robot_controller/button'
  process = subprocess.Popen(
    ['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True
  )

  while True:
    output = process.stdout.readline()
    if output:
      line = output.strip()
      if line.startswith("id:"):
        button_id = int(line.split(":")[1].strip())
      elif line.startswith("state:"):
        state = int(line.split(":")[1].strip())

        if button_queue.full(): continue
        button_queue.put((button_id, state))
    else:
      continue

def check_node_status():
  command = 'source /home/ubuntu/.zshrc && ros2 topic list'
  result = subprocess.run(['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command], capture_output=True, text=True, check=False)
  res = result.stdout
  return '/ros_robot_controller/button' in res

if check_node_status():
  print("ROS2 node detected")
  listener_thread = threading.Thread(target=listen_to_button_events, daemon=True)
  listener_thread.start()
