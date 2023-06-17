#!/usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from grid_map_msgs.msg import GridMap

# topic = '/elevation_mapping/elevation_map'
topic = '/elevation_mapping/elevation_map_raw'

class EnvElevationMap:
  def __init__(self):
    rospy.init_node("ElevationGridmapEnv")
    self.elevation_map = None
    self.elevation_matrix = None
    self.full_elevation_matrix = None
    self.robot_pose = None
    self.x_range = None
    self.y_range = None

    self.elevation_map_subscriber = rospy.Subscriber(topic, GridMap, self.elevation_map_callback) 

    self.marker_pub = rospy.Publisher('/elevation_nodes', Marker, queue_size=10)
    self.marker = Marker()
    self.marker.header.frame_id = "world"
    self.marker.type = Marker.POINTS
    self.marker.action = Marker.ADD 
    self.marker.pose.orientation.w = 1.0
    self.marker.scale.x = 0.1
    self.marker.scale.y = 0.1
    self.marker.color.a = 1.0
    self.marker.color.r = 1.0
    rospy.sleep(1)

    rospy.wait_for_message(topic, GridMap)

  def elevation_map_callback(self, grid_map):
    self.elevation_map = grid_map.data[0]
    self.robot_pose = grid_map.info.pose
    self.x_range = int(grid_map.info.length_x / grid_map.info.resolution)
    self.y_range = int(grid_map.info.length_y / grid_map.info.resolution)
    self.resolution = grid_map.info.resolution

    self.robot_pos = grid_map.info.pose

    self.column_size = self.elevation_map.layout.dim[0].size
    self.row_size = self.elevation_map.layout.dim[1].size
    # print(self.elevation_map.layout.dim[1].label + " has size " + str(self.row_size))
    self.row_stride = self.elevation_map.layout.dim[1].stride

    self.row_start = grid_map.outer_start_index
    self.column_start = grid_map.inner_start_index

    elevation_list = []
    for i in range(self.row_size):
      for j in range(self.column_size):
        index = self.row_stride * i + j
        height = self.elevation_map.data[index]
        elevation_list.append(height)

    # start = self.column_start * self.row_stride + self.row_start
    # self.elevation_map = elevation_list[start::1] + elevation_list[0:start]
    # print(self.elevation_map)
    self.elevation_map = elevation_list

    self.elevation_matrix = [[0] * self.row_size for _ in range(self.column_size)]
    for i in range(self.row_size):
      for j in range(self.column_size):
        index = self.row_stride * i + j
        height = self.elevation_map[index]
        if math.isnan(height):
          self.elevation_matrix[i][j] = -1
        else:
          self.elevation_matrix[i][j] = round(height, 2)
    
    self.marker.points = []
    self.elevation_matrix = self.fix(np.array(self.elevation_matrix), self.row_start, self.column_start)
    self.rot_matrix = np.rot90(np.flipud(self.elevation_matrix))

    for x in range(self.row_size):
      for y in range(self.column_size):
        height = self.rot_matrix[x][y]
        # height = self.elevation_matrix[x][y]
        self.marker.points.append(Point(self.scale(x, 0, self.column_size, -1.5, 1.5) + (self.resolution / 2) + self.robot_pos.position.x, 
                                        self.scale(y, 0, self.row_size, -1.5, 1.5) + (self.resolution / 2) + self.robot_pos.position.y,
                                          height))

    self.marker_pub.publish(self.marker) 

  def scale(self, val, x1, y1, x2, y2):
    output = (val - x1) * (y2 - x2) / (y1 - x1) + x2
    return round(output, 1)
  
  def fix(self, matrix, outer, inner):
    matrix = np.roll(matrix.T, -outer, axis=1).T
    matrix = np.roll(matrix, -inner, axis=1)
    return matrix

if __name__ == '__main__':
  env = EnvElevationMap()
  print("elevation matrix")
  for row in env.elevation_matrix:
    print(row)
  print("rotation matrix")
  for row in env.rot_matrix:
    print(row)
  while not rospy.is_shutdown():
    env.marker_pub.publish(env.marker)

