#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import xml.etree.ElementTree as ET
from enum import Enum
import random

num = 0

class map_state(Enum):
  AVAIL = 0,
  OBS = 1

# each world it 30 * 64 by the number of cylinders. 
rows, cols = (30, 64)
the_map = []
for i in range(0, 30):
  this_row = []
  for i in range(0,64):
    this_row.append(map_state.AVAIL)
  the_map.append(this_row)

def aroud_obs():
  pass

def callback(msg):
  pos = msg.pose.pose.position
  print(f"({pos.x:.5f},{pos.y:.5f},{pos.z:.5f})")

def listener():
  rospy.init_node('listener', anonymous=True)
  
  rospy.Subscriber("/odometry/filtered", Odometry, callback)

  rospy.spin()

def get_end_point():
  global the_map
  tree = ET.parse("/home/never0lie/research/22spring/robot/BARN_dataset/world_files/world_0.world")
  root = tree.getroot()
  world = root.find("world")
  obs_poses = []
  for model in world.findall("model"):
    if model.attrib.get('name') == "ground_plane": continue 
    # obs_pose = list(map(lambda s: float(s), ))
    obs_pose = model.find("pose").text.split(" ")
    obs_pose = obs_pose[:2]
    # print(obs_pose)
    obs_poses.append(obs_pose)
  # print(len(obs_poses))
  obs_poses = map(lambda x: [float(x[0]), float(x[1])],obs_poses)
  obs_index_poses = list(map(lambda x: [round((-x[0] - 0.075) / 0.15), round((x[1] - 0.075) / 0.15)], obs_poses))
  # print(obs_index_poses)
  for coordinate in obs_index_poses:
    # print(coordinate)
    r = coordinate[0]
    c = coordinate[1]
    the_map[r][c] = map_state.OBS
  # print(the_map)
  continue_generating = True
  r = -1
  c = -1
  while (continue_generating):
    continue_generating = False
    r = random.randint(0, 29)
    c = random.randint(0, 63)
    if the_map[r][c] == map_state.OBS:
      continue_generating = True
  return (-(0.15 * r + 0.075), 0.15 * c + 0.075)

if __name__ == '__main__':
  print(get_end_point())
  # listener()