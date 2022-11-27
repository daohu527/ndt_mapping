#!/usr/bin/env python

# Copyright 2022 daohu527 <daohu527@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import bisect
import sys
import os
from pathlib import Path

from cyber_record.record import Record
from record_msg.parser import PointCloudParser

pose_index = 1
pcd_index = 1

pose_timestamps = []
pose_contents = []

pcd_indexs = []
pcd_timestamps = []

def parse_pose(msg):
  global pose_index
  timestamp = msg.measurement_time

  x = msg.pose.position.x
  y = msg.pose.position.y
  z = msg.pose.position.z

  qx = msg.pose.orientation.qx
  qy = msg.pose.orientation.qy
  qz = msg.pose.orientation.qz
  qw = msg.pose.orientation.qw

  std_x = msg.uncertainty.position_std_dev.x
  std_y = msg.uncertainty.position_std_dev.y
  std_z = msg.uncertainty.position_std_dev.z

  line = "{} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
    pose_index, timestamp, x, y, z, qx, qy, qz, qw, std_x, std_y, std_z)
  f_fusion_loc.write(line)

  pose_timestamps.append(timestamp)
  pose_contents.append("{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}".format(
    x, y, z, qx, qy, qz, qw))
  pose_index += 1



def parse_pointcloud(msg):
  global pcd_index
  pointcloud_parser.parse(msg)
  f_pcd_timestamp.write("{} {:.6f}\n".format(pcd_index, msg.measurement_time))
  pcd_index += 1

  pcd_indexs.append(pcd_index)
  pcd_timestamps.append(msg.measurement_time)


if __name__ == "__main__":
  args=sys.argv

  parser = argparse.ArgumentParser(
    description="cyber_record_parser is a cyber record file offline parse tool.",
    prog="main.py")

  parser.add_argument(
    "-f", "--file", action="store", type=str, required=True,
    nargs='?', const="", help="cyber record file")
  parser.add_argument(
    "-t", "--topic", action="store", type=str, required=False,
    nargs='?', const="/apollo/sensor/lidar32/compensator/PointCloud2",
    help="cyber message topic")

  args = parser.parse_args(args[1:])


  Path("data/pcd").mkdir(parents=True, exist_ok=True)

  f_fusion_loc = open("data/pcd/fusion_loc.txt", 'a')
  f_pcd_timestamp = open("data/pcd/pcd_timestamp.txt", 'a')
  pointcloud_parser = PointCloudParser('data/pcd')

  # bag
  if os.path.isfile(args.file):
    print("Start to parse record to 'data/pcd', pls wait!")
    record = Record(args.file)
    for topic, message, t in record.read_messages_fallback():
      if topic == "/apollo/localization/pose":
        parse_pose(message)
      elif topic == args.topic:
        parse_pointcloud(message)
      else:
        pass
  else:
    print("File not exist! {}".format(args.file))

  f_fusion_loc.close()
  f_pcd_timestamp.close()


  i = 1
  f_pose = open("data/pcd/poses.txt", 'a')
  for pcd_timestamp in pcd_timestamps:
    index = bisect.bisect(pose_timestamps, pcd_timestamp)
    data = pose_contents[index]
    f_pose.write("{} {} {}\n".format(i, pcd_timestamp, data))
    i += 1
  f_pose.close()
