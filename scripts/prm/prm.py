#!/usr/bin/env python3 

import rospy
import math
import random
import heapq

from nav_msgs.msg import Path  
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from sklearn.neighbors import KNeighborsRegressor

from util import Util

topic = 'visualization_marker_array'

class PRM:
  """
  Documentation here
  """
  def __init__(self, start: tuple, goal: tuple, sample_nodes: int, k: int):
    self.init = rospy.init_node("prm")
    self.frame = "world"

    self.start_point = start
    self.goal_point = goal
    self.sample_nodes = sample_nodes
    self.k = k
    self.x_range = [-4, 4]
    self.y_range = [-4, 4]

    self.markers = []
    self.marker_subscriber = rospy.Subscriber(topic, MarkerArray, self.marker_callback)
    rospy.wait_for_message(topic, MarkerArray)
    # rospy.sleep(0.4)
    self.start_node, self.goal_node = self.get_initial_nodes(self.markers)
    self.x_range, self.y_range = self.get_xy_range(self.markers)
        
    self.z_range = (0, 0.75) #highest rectangle obstacle height is 0.5 + 0.25 (z-center)

    self.obs_boundary = self.get_boundary(self.markers)
    self.obs_rectangle = self.get_obs_rectangle(self.markers)
    self.obs_circle = self.get_obs_cylinder(self.markers)

    self.utils = Util(self.obs_circle, self.obs_rectangle, self.obs_boundary)

    self.path_pub = rospy.Publisher("/path", Path, queue_size=10)

    self.marker_pub = rospy.Publisher('/nodes', Marker, queue_size=10)
    self.marker = Marker()
    self.marker.header.frame_id = self.frame
    self.marker.type = Marker.POINTS
    self.marker.action = Marker.ADD 
    self.marker.pose.orientation.w = 1.0
    self.marker.scale.x = 0.1
    self.marker.scale.y = 0.1
    self.marker.color.a = 1.0
    self.marker.color.r = 1.0
    self.marker.points.append(Point(start[0], start[1], start[2]))

    self.marker_graph_pub = rospy.Publisher('/graph', Marker, queue_size=10)
    self.marker_graph = Marker()
    self.marker_graph.header.frame_id = self.frame
    self.marker_graph.type = Marker.LINE_LIST
    self.marker_graph.action = Marker.ADD 
    self.marker_graph.pose.orientation.w = 1.0
    self.marker_graph.scale.x = 0.01
    self.marker_graph.scale.y = 0.01
    self.marker_graph.color.a = 1.0
    self.marker_graph.color.r = 1.0

    self.marker_dest_pub = rospy.Publisher('/dest', Marker, queue_size=10)
    self.marker_dest = Marker()
    self.marker_dest.header.frame_id = self.frame
    self.marker_dest.type = Marker.POINTS
    self.marker_dest.action = Marker.ADD 
    self.marker_dest.pose.orientation.w = 1.0
    self.marker_dest.scale.x = 0.3
    self.marker_dest.scale.y = 0.3
    self.marker_dest.color.a = 1.0
    self.marker_dest.color.r = 1.0
    self.marker_dest.color.g = 1.0

    self.marker_dest.points.append(Point(start[0], start[1], start[2]))
    self.marker_dest.points.append(Point(goal[0], goal[1], goal[2]))

    self.points = [self.start_point, self.goal_point]
  
  def planner(self):
    nodes = self.generate_nodes()
    neighbors = self.find_k_closest_neighbors(nodes, self.k)
    self.draw_graph(neighbors)
    path = self.find_shortest_path(neighbors, self.start_point, self.goal_point)
    return path
  
  def draw_graph(self, neighbors: dict):
    for point in neighbors:
      for neighbor in neighbors[point]:
        self.marker_graph.points.append(Point(point[0], point[1], 0))
        self.marker_graph.points.append(Point(neighbor[0], neighbor[1], 0))
      self.marker_graph_pub.publish(self.marker_graph)

  def find_k_closest_neighbors(self, points: list, k: int) -> dict:
    X = [[x, y, z] for x, y, z in points]
    knn = KNeighborsRegressor(n_neighbors=k)
    knn.fit(X, X)  
    k_neighbors = knn.kneighbors(X, return_distance=False)
    neighbors_dict = {}

    for i, neighbors in enumerate(k_neighbors):
      current_point = points[i]
      closest_neighbors = [points[n] for n in neighbors if n != i]
      for node_near in closest_neighbors:
        if self.utils.is_collision(node_near, current_point):
          closest_neighbors.remove(node_near)
      neighbors_dict[current_point] = closest_neighbors
    return neighbors_dict

  def generate_nodes(self):
    for i in range(self.sample_nodes):
      x = round(random.uniform(self.x_range[0], self.x_range[1]), 2)
      y = round(random.uniform(self.y_range[0], self.y_range[1]), 2)
      new_point = (x, y, 0)
      if self.utils.is_inside_obs(new_point):
        continue
      else:
        self.marker.points.append(Point(x, y, 0))
        self.points.append(new_point)
    return self.points

  def find_shortest_path(self, graph: dict, start: tuple, goal: tuple):
    if goal not in graph:
        return None
    
    path = {start: (0, None)}
    pq = [(0, start)]
    
    while pq:
      current_distance, current_node = heapq.heappop(pq)
      if current_distance > path[current_node][0]:
        continue
        
      if current_node == goal:
        break
        
      for neighbor in graph[current_node]:
        distance = current_distance + math.sqrt((current_node[0] - neighbor[0]) ** 2 + (current_node[1] - neighbor[1]) ** 2)

        if neighbor not in path or distance < path[neighbor][0]:
          path[neighbor] = (distance, current_node)
          heapq.heappush(pq, (distance, neighbor))
    
    if goal not in path:
      return None
    
    shortest_path = []
    current_node = goal
    
    while current_node is not None:
      shortest_path.append(current_node)
      _, current_node = path[current_node]
    
    shortest_path.reverse()
    return shortest_path

  def plot_path(self, nodes: list):
    path_msg = Path()
    path_msg.header.frame_id = self.frame

    for tup in nodes:
      pose = PoseStamped()
      pose.header.frame_id = self.frame
      pose.pose.position = Point(tup[0], tup[1], 0)
      path_msg.poses.append(pose)
        
    self.path_pub.publish(path_msg)
    return path_msg
  
  def marker_callback(self,data):
    self.markers=data
        
    self.marker_subscriber.unregister()
    rospy.sleep(1)

  def get_xy_range(self,markers):
    corners= (markers.markers[0].points) #get the boundary coordinates
    x_array=[]
    y_array=[]

    for i in range(len(corners)):
        x_array.append(corners[i].x)
        y_array.append(corners[i].y)

    x_range=(min(x_array),max(x_array))
    y_range=(min(y_array),max(y_array))

    return x_range,y_range

  def get_boundary(self,markers):
    corners = (markers.markers[0].points) # get the boundary coordinates
    corners_list=[] # (ox,oy,w,h)*4 - I considered the complete Z 
    for i in range(len(corners)-1):
        if i==0 or i==2:
            w=(corners[i+1].x-corners[i].x)
            h=0.2
        elif i==1 or i==3:
            w=0.2
            h=(corners[i+1].y-corners[i].y)
        corners_list.append([corners[i].x,corners[i].y,w,h])
    return corners_list
            

  def get_initial_nodes(self,markers):
    initial_points=(markers.markers[-1].points) # last Marker added are the start/end nodes
    start_node=(initial_points[0].x,initial_points[0].y,initial_points[0].z)
    goal_node=(initial_points[1].x,initial_points[1].y,initial_points[1].z)
        
    return start_node,goal_node

  def get_obs_rectangle(self, markers):
    # we have four obstacles from markers[1 -> 4 -included]
    # z_center is considered 0 and not 0.25 - We don't want the robot to go below the obstacle
    rectangle_arr=[]

    #Obstacle 1
    x_1_center = markers.markers[1].pose.position.x
    y_1_center = markers.markers[1].pose.position.y
    z_1_center = markers.markers[1].pose.position.z

    x_1 = x_1_center - markers.markers[1].scale.x / 2.0
    y_1 = y_1_center - markers.markers[1].scale.y /2.0
    #z_1 = z_1_center
    z_1=0 

    w_1 = markers.markers[1].scale.x+ 0.1
    h_1 = markers.markers[1].scale.y+ 0.1
    l_1 = markers.markers[1].scale.z+ 0.1
    rectangle_arr.append([x_1,y_1,z_1,w_1,h_1,l_1])

    # Obstacle 2
    x_2_center = markers.markers[2].pose.position.x
    y_2_center = markers.markers[2].pose.position.y
    z_2_center = markers.markers[2].pose.position.z

    x_2 = x_2_center - markers.markers[2].scale.y / 2.0
    y_2 = y_2_center - markers.markers[2].scale.x /2.0
    #z _2 = z_2_center
    z_2=0 

    w_2 = markers.markers[2].scale.y
    h_2 = markers.markers[2].scale.x
    l_2 = markers.markers[2].scale.z
    rectangle_arr.append([x_2,y_2,z_2,w_2,h_2,l_2])

    # Obstacle 3
    x_3_center = markers.markers[3].pose.position.x
    y_3_center = markers.markers[3].pose.position.y
    z_3_center = markers.markers[3].pose.position.z

    x_3 = x_3_center - markers.markers[3].scale.x / 2.0
    y_3 = y_3_center - markers.markers[3].scale.y / 2.0
    # z_3 = z_3_center
    z_3 = 0

    w_3 = markers.markers[3].scale.x
    h_3 = markers.markers[3].scale.y
    l_3 = markers.markers[3].scale.z
    rectangle_arr.append([x_3,y_3,z_3,w_3,h_3,l_3])

    #Obstacle 4
    x_4_center = markers.markers[4].pose.position.x
    y_4_center = markers.markers[4].pose.position.y
    z_4_center = markers.markers[4].pose.position.z

    x_4 = x_4_center - markers.markers[4].scale.y / 2.0
    y_4 = y_4_center - markers.markers[4].scale.x / 2.0
    # z_4 = z_4_center
    z_4 = 0

    w_4 = markers.markers[4].scale.y + 0.1
    h_4 = markers.markers[4].scale.x + 0.1
    l_4 = markers.markers[4].scale.z + 0.1
    rectangle_arr.append([x_4,y_4,z_4,w_4,h_4,l_4])
        
    return rectangle_arr

  def get_obs_cylinder(self, markers):
    cylinder_arr=[]

    for i in range(5,8):
        x = markers.markers[i].pose.position.x
        y = markers.markers[i].pose.position.y
        z = markers.markers[i].pose.position.z

        r = markers.markers[i].scale.x / 2.0 # Radius
        h = markers.markers[i].scale.z  # Height

        cylinder_arr.append([x,y,z,r,h])

    return cylinder_arr


def main():
  start = (-4, -4, 0)
  goal = (4, 4, 0)
  sample = 1000
  k = 20
  prm = PRM(start, goal, sample, k)
  path = prm.planner()
  if path:
    print("Path Found!")
    while not rospy.is_shutdown():
      prm.plot_path(path)
      prm.marker_pub.publish(prm.marker)
      prm.marker_graph_pub.publish(prm.marker_graph)
      prm.marker_dest_pub.publish(prm.marker_dest)
  else:
    print("Path Not Found!")

if __name__ == '__main__':
  main()
