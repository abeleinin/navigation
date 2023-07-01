#!/usr/bin/env python3

import os
import math
import numpy as np

import rospy
import ros_numpy

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from grid_map_msgs.msg import GridMap

import yaml
from yaml.loader import SafeLoader


elevation_topic = '/elevation_map_raw_visualization/elevation_cloud'
map_region_topic = '/elevation_map_raw_visualization/map_region'

class elevationMap:
  def __init__(self):
    self.resolution = None
    self.x_length = None
    self.y_length = None

    self.get_map_params()

    self.x_range = self.x_length / self.resolution
    self.y_range = self.y_length / self.resolution

    self.map_region = []
    self.elevation_matrix = None
    self.array = None

    self.x_min = None
    self.x_max = None
    self.y_min = None
    self.y_max = None

    self.marker_pub = rospy.Publisher('/elevation_nodes', Marker, queue_size=10)
    self.marker = Marker()
    self.marker.header.frame_id = 'world'
    self.marker.type = Marker.POINTS
    self.marker.action = Marker.ADD 
    self.marker.pose.orientation.w = 1.0
    self.marker.scale.x = 0.1
    self.marker.scale.y = 0.1
    self.marker.color.a = 1.0
    self.marker.color.r = 1.0

    self.elevation_map_region_sub = rospy.Subscriber(map_region_topic, Marker, self.map_region_callback)
    rospy.loginfo('Waiting for map region...')
    rospy.wait_for_message(map_region_topic, Marker)
    rospy.loginfo('World x min/max ' + str(self.x_min) + ', ' + str(self.x_max))
    rospy.loginfo('World y min/max ' + str(self.y_min) + ', ' + str(self.y_max))
    self.elevation_map_subscriber = rospy.Subscriber(elevation_topic, PointCloud2, self.elevation_points_callback, queue_size=1)
    rospy.loginfo('Waiting for PointCloud...')
    rospy.wait_for_message(elevation_topic, PointCloud2)

  # Get parameters from elevation map yaml file
  def get_map_params(self):
    curr_dir = os.getcwd()
    path = os.path.join(curr_dir, '..', '..', 'config/elevation_maps/simple_demo_map.yaml')
    with open(path) as f:
        data = yaml.load(f, Loader=SafeLoader)
        self.resolution = data['resolution']
        self.x_length = data['length_in_x']
        self.y_length = data['length_in_y']

  # Retrieve map dimensions 
  def map_region_callback(self, marker):
    # Optimize later
    self.x_min = round(min(point.x for point in marker.points) + self.resolution / 2, 2)
    self.x_max = round(max(point.x for point in marker.points) - self.resolution / 2, 2)
    self.y_min = round(min(point.y for point in marker.points) + self.resolution / 2, 2)
    self.y_max = round(max(point.y for point in marker.points) - self.resolution / 2, 2)
    self.elevation_map_region_sub.unregister()

  def elevation_points_callback(self, pc2_msg):
    array = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
    # print(self.x_min, self.y_min, self.x_max, self.y_max)

    # self.marker.points = []
    matrix = [[0] * int(self.y_range) for _ in range(int(self.x_range))]
    for point in array:
      # self.marker.points.append(Point(point[0], point[1], point[2]))
      i = self.scale(round(point[0].item(), 2), self.x_min, self.x_max, 0, self.x_range - 1)
      j = self.scale(round(point[1].item(), 2), self.y_min, self.y_max, 0, self.x_range - 1)
      height = round(point[2].item(), 2)
      # print(point[0].item())
      # print(point[1].item())
      # print(int(i), int(j))
      matrix[int(i)][int(j)] = height
    
    self.elevation_matrix = matrix
    # self.elevation_matrix = np.fliplr(np.flipud(matrix))
    # self.print_matrix(self.elevation_matrix)

    # self.marker_pub.publish(self.marker) 
    rospy.loginfo('Elevation matrix created...')
    # rospy.loginfo('Elevation matrix created. Next update in 3 sec...')
    # rospy.sleep(1)
    self.elevation_map_subscriber.unregister()

  # Linear interpolation function 
  def scale(self, val, x1, y1, x2, y2):
    output = (val - x1) * (y2 - x2) / (y1 - x1) + x2
    return round(output, 1)

  # Print matrix
  def print_matrix(self, matrix):
    for row in matrix: 
      print(row)
    print('------------------------------')

# Run for testing
if __name__ == '__main__':
  env = elevationMap()
  while not rospy.is_shutdown():
    env.marker_pub.publish(env.marker)

