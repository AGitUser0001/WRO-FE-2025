#!/usr/bin/env python3
import subprocess, threading, math, multiprocessing

CONTAINER = "MentorPi"
SHELL     = "/bin/zsh"
USER      = "ubuntu"
WORKDIR   = "/home/ubuntu"
ENV_SRC   = "source /home/ubuntu/.zshrc"
TOPIC     = "/imu/rpy/filtered"  # geometry_msgs/Vector3Stamped
imu_value = multiprocessing.Value("d", 0.0)

def check_topic_exists():
  cmd = f"{ENV_SRC} && ros2 topic list"
  r = subprocess.run(
    ["docker","exec","-u",USER,"-w",WORKDIR,CONTAINER,SHELL,"-c",cmd],
    capture_output=True, text=True, check=False
  )
  return TOPIC in r.stdout

def listen_to_imu_z(imu_value):
  # Stream the topic and parse only the 'vector.z' line
  cmd = f"{ENV_SRC} && ros2 topic echo {TOPIC}"
  p = subprocess.Popen(
    ["docker","exec","-u",USER,"-w",WORKDIR,CONTAINER,SHELL,"-c",cmd],
    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
  )

  first_value = -1
  in_vector_block = False
  for line in p.stdout:
    s = line.strip()
    if not s:
      continue
    if s.startswith("vector:"):
      in_vector_block = True
      continue
    if in_vector_block:
      if s.startswith("z:"):
        try:
          z_rad = float(s.split(":",1)[1].strip())
          z_deg = math.degrees(z_rad) % 360
          if first_value == -1:
            first_value = z_deg
          z_deg -= first_value
          imu_value.value = z_deg
        except ValueError:
          pass
        in_vector_block = False
      elif s[0].isalpha() and not s.startswith(("x:","y:")):
        # new section started; leave vector block
        in_vector_block = False

if check_topic_exists():
  print("IMU topic detected; listening for yaw (z)…")
  t = threading.Thread(target=listen_to_imu_z, args=(imu_value,), daemon=True)
  t.start()
else:
  print(f"Topic {TOPIC} not found.")
