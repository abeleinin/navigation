#!/usr/bin/env python3

import math
import random
import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class Node:
    def __init__(self, point):
      self.point = point # Point
      self.angle = 0
      self.parent = None

class RRT:
    def __init__(self, step_len, iter_max):
      self.init = rospy.init_node("rrt")
      self.frame = "world"

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

      self.marker_tree_pub = rospy.Publisher('/tree', Marker, queue_size=10)
      self.marker_tree = Marker()
      self.marker_tree.header.frame_id = self.frame
      self.marker_tree.type = Marker.LINE_LIST
      self.marker_tree.action = Marker.ADD 
      self.marker_tree.pose.orientation.w = 1.0
      self.marker_tree.scale.x = 0.01
      self.marker_tree.scale.y = 0.01
      self.marker_tree.color.a = 1.0
      self.marker_tree.color.r = 1.0

      self.start_point = Point(0, 0, 0) # Start
      self.goal_point = Point(-10, -10, 0) # Goal

      self.start_node = Node(self.start_point)
      self.goal_node = Node(self.goal_point)
      self.marker.points.append(self.start_point)

      self.step_len = step_len
      self.iter_max = iter_max
      self.angles = [-45, 0, 45] # Sprouting angles 
      self.nodes = [self.start_node]
      self.curr_parent = self.start_node
      self.leaves = []
      self.rev_leaves = [self.start_node]
     
    def planning(self):
      count = 0
      for i in range(self.iter_max):
        if i % 100 == 0:
          print(i)
        if math.isqrt(count) ** 2 == count:
          self.step_len *= 1
        for it in range(3):
          angle = random.randint(-60, 60)
          x = self.curr_parent.point.x + self.step_len * math.cos(math.radians(angle + self.curr_parent.angle))
          y = self.curr_parent.point.y + self.step_len * math.sin(math.radians(angle + self.curr_parent.angle))
          new_node = Node(Point(x, y, 0))
          new_node.parent = self.curr_parent
          new_node.angle = angle + self.curr_parent.angle
          self.nodes.append(new_node)
          self.marker_tree.points.append(new_node.point)
          self.marker_tree.points.append(new_node.parent.point)
          self.marker_tree_pub.publish(self.marker_tree)
          rospy.sleep(0.01)
          dist, _ = self.get_distance_and_angle(new_node, self.goal_node)

          if dist <= self.step_len:
            dist, theta = self.get_distance_and_angle(new_node, self.goal_node)

            dist = min(self.step_len, dist)
            point_new = Point()
            point_new.x = new_node.point.x + dist * math.cos(theta)
            point_new.y = new_node.point.y + dist * math.sin(theta)
            point_new.z = 0

            final = Node(point_new)
            final.parent = new_node
            return self.extract_path(new_node)

          self.leaves.append(new_node)
          count += 1 
          
        self.curr_parent = self.leaves.pop(0)
        # self.curr_parent = self.leaves.pop(random.randint(0, len(self.leaves) - 1))

      return None
    
    def extract_path(self, node_end):
      goal_point = Point(self.goal_node.point.x, self.goal_node.point.y, self.goal_node.point.z)
      goal = Node(goal_point)
      path = [goal]
      node_now = node_end

      while node_now.parent is not None:
        node_now = node_now.parent
        self.marker.points.append(node_now.point)
        new_point = Point(node_now.point.x, node_now.point.y, node_now.point.z)
        new_node = Node(new_point)
        path.append(new_node)

      return path

    def get_distance_and_angle(self, node_start, node_end):
      dx = node_end.point.x - node_start.point.x
      dy = node_end.point.y - node_start.point.y
      return math.hypot(dx, dy), math.atan2(dy, dx)
      

def main():
  rrt = RRT(0.75, 1000)
  path = rrt.planning()
  if path:
    print("Path Found!")
    while not rospy.is_shutdown():
       rrt.marker_tree_pub.publish(rrt.marker_tree)
       rrt.marker_pub.publish(rrt.marker)
      
if __name__ == '__main__':
  main()