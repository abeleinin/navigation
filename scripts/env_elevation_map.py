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
    self.x_range = 0
    self.y_range = 0

    self.map_x_range = 5
    self.map_y_range = 5

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
    rospy.wait_for_message(topic, GridMap)

  def elevation_map_callback(self, grid_map):
    self.elevation_map = grid_map.data[0]
    self.robot_pose = grid_map.info.pose
    self.x_range = int(grid_map.info.length_x / grid_map.info.resolution)
    self.y_range = int(grid_map.info.length_y / grid_map.info.resolution)
    self.resolution = int(grid_map.info.resolution)

    # if self.elevation_map != None:
    #   rospy.loginfo("Elevation Map received")

    matrix = []
    for x in range(self.x_range):
      row = []
      for y in range(self.y_range):
        index = 100 * x + y
        row.append(self.elevation_map.data[index])
      matrix.append(row)

    full_matrix = []

    for x in range(self.x_range):
      row = []
      high_value = 10
      nan_cnt = 0
      for y in range(self.y_range):
        if math.isnan(matrix[x][y]):
          nan_cnt += 1
        else:
          high_value = min(high_value, matrix[x][y])
          while nan_cnt > 0:
            row.append(high_value)
            nan_cnt -= 1
          row.append(high_value)
      while nan_cnt > 0:
        row.append(np.nan)
        # row.append(1)
        nan_cnt -= 1
      full_matrix.append(row)
    
    matrix = np.rot90(np.flipud(matrix))
    matrix = np.rot90(np.flipud(full_matrix))
    
    for x in range(self.x_range):
      for y in range(self.y_range):
        height = matrix[x][y]
        if not math.isnan(height):
          point = Point()
          point.x = self.scale(x, 0, 99, -5, 5) 
          point.y = self.scale(y, 0, 99, -5, 5)
          point.z = height
          self.marker.points.append(point)

    self.elevation_matrix = matrix
    self.full_elevation_matrix = full_matrix

    self.marker_pub.publish(self.marker)
    # rospy.loginfo("Gridmap points published")
    

  def is_collision(self, point_near, point_new):
    return True 
  
# Linear interpolation formula
  def scale(self, val, x1, y1, x2, y2):
    output = (val - x1) * (y2 - x2) / (y1 - x1) + x2
    return round(output, 1)

if __name__ == '__main__':
  env = EnvElevationMap()
  while not rospy.is_shutdown():
    env.marker_pub.publish(env.marker)

