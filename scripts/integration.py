#!/usr/bin/env python3

import math
import random
import rospy

import tf
from nav_msgs.msg import Path  
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from visualization_msgs.msg import Marker

from env_matrix import EnvElevationMap
from robot_odom import robotOdom

class Node:
    def __init__(self, point):
      self.point = point # Point
      self.angle = 0
      self.parent = None

class RRT:
    def __init__(self, step_len, iter_max):
      self.env = EnvElevationMap()
      self.jackal = robotOdom()
      # self.init = rospy.init_node("rrt")
      self.frame = "world"

      self.path_pub = rospy.Publisher("/world/path", Path, queue_size=10)

      # q = tf.transformations.quaternion_from_euler(0, 0, 1)
      # world_quaternion = Quaternion(q[0], q[1], q[2], q[3])

      self.marker_pub = rospy.Publisher('/world/nodes', Marker, queue_size=10)
      self.marker = Marker()
      self.marker.header.frame_id = self.frame
      self.marker.type = Marker.POINTS
      self.marker.action = Marker.ADD 
      self.marker.pose.orientation.w = 1.0
      self.marker.scale.x = 0.1
      self.marker.scale.y = 0.1
      self.marker.color.a = 1.0
      self.marker.color.g = 1.0

      self.marker_tree_pub = rospy.Publisher('/world/tree', Marker, queue_size=10)
      self.marker_tree = Marker()
      self.marker_tree.header.frame_id = self.frame
      self.marker_tree.type = Marker.LINE_LIST
      self.marker_tree.action = Marker.ADD 
      self.marker_tree.pose.orientation.w = 1.0
      self.marker_tree.scale.x = 0.03
      self.marker_tree.scale.y = 0.03
      self.marker_tree.color.a = 1.0
      self.marker_tree.color.r = 1.0
      self.marker_tree.color.g = 0.6

      self.marker_dest_pub = rospy.Publisher('/world/dest', Marker, queue_size=10)
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

      self.testing_pub = rospy.Publisher('/world/bresenham', Marker, queue_size=10)
      self.testing = Marker()
      self.testing.header.frame_id = self.frame
      self.testing.type = Marker.POINTS
      self.testing.action = Marker.ADD 
      self.testing.pose.orientation.w = 1.0
      self.testing.scale.x = 0.1
      self.testing.scale.y = 0.1
      self.testing.color.a = 1.0
      self.testing.color.b = 1.0

      self.start_point = self.jackal.position # Robot position 
      self.goal_point = Point(5, -5, 0) # Goal

      self.marker_dest.points.append(self.start_point)
      self.marker_dest.points.append(self.goal_point)

      self.start_node = Node(self.start_point)
      self.goal_node = Node(self.goal_point)
      self.marker.points.append(self.start_point)

      self.map_x_range = [self.env.x_min, self.env.x_max]
      self.map_y_range = [self.env.y_min, self.env.y_max]
      self.limit = 0.1

      self.step_len = step_len
      self.iter_max = iter_max
      self.nodes = [self.start_node]
      self.front_parent = self.start_node
      self.back_parent = self.start_node
      self.front_leaves = []
      self.back_leaves = []
     
    def planning(self):
      count = 1
      for i in range(self.iter_max):
        if i % 100 == 0:
          print(i)
        # if math.log(count, 3).is_integer():
        #   self.step_len *= 1

        # Find path in front of robot or behind 
        if self.front_parent != None:
          front_path = self.generate_leafs(3, 0 + self.jackal.yaw, self.front_parent, self.front_leaves)
        if self.back_parent != None:
          back_path = self.generate_leafs(3, 180 + self.jackal.yaw, self.back_parent, self.back_leaves)

        # Check if a path was found
        if front_path != None:
          return front_path
        elif back_path != None:
          return back_path

        # Random Leaf Node Selection 
        # Determine the next parent node. If leaves list is empty then all leaves lead to a collision.
        self.front_parent = self.front_leaves.pop(random.randint(0, len(self.front_leaves) - 1)) if self.front_leaves else None
        self.back_parent = self.back_leaves.pop(random.randint(0, len(self.back_leaves) - 1)) if self.back_leaves else None

        # Sequential Leaf Node Selection
        # self.front_parent = self.front_leaves.pop(0)
        # self.back_parent = self.back_leaves.pop(0)

        count += 1

      return None
    
    def generate_leafs(self, num, direction, curr_parent, leaves):
      delta = 15
      angles = [] # Random angles 
      while len(angles) < num:
        new_angle = random.randint(-45, 45)
        if all(abs(new_angle - existing_angle) >= delta for existing_angle in angles):
          angles.append(new_angle)
      # angles = [-60, -45, 0, 45, 60] # Set angels
      for angle in angles:
        x = curr_parent.point.x + self.step_len * math.cos(math.radians(angle + curr_parent.angle + direction))
        y = curr_parent.point.y + self.step_len * math.sin(math.radians(angle + curr_parent.angle + direction))
        new_node = Node(Point(x, y, 0.05))
        new_node.parent = curr_parent
        new_node.angle = angle + curr_parent.angle
        collision = self.is_collision(new_node)
        if collision:
          print("collision")
          return None
        self.nodes.append(new_node)
        self.marker_tree.points.append(Point(new_node.point.x, new_node.point.y, 0.05))
        self.marker_tree.points.append(Point(new_node.parent.point.x, new_node.parent.point.y, 0.05))
        self.marker_tree_pub.publish(self.marker_tree)
        self.marker_dest_pub.publish(self.marker_dest)
        rospy.sleep(0.04)
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
          self.marker_tree.points.append(new_node.parent.point)
          self.marker_tree.points.append(final.point)
          return self.extract_path(new_node)
        leaves.append(new_node)
      return None

    def coord_to_matrix(self, node):
        x = round(self.env.scale(node.point.x, self.map_x_range[0], self.map_x_range[1], 0, 99))
        y = round(self.env.scale(node.point.y, self.map_y_range[0], self.map_y_range[1], 0, 99))
        parent_x = round(self.env.scale(node.parent.point.x, self.map_x_range[0], self.map_x_range[1], 0, 99))
        parent_y = round(self.env.scale(node.parent.point.y, self.map_y_range[0], self.map_y_range[1], 0, 99))
        new_point = Point(x, y, 0)
        parent = Point(parent_x, parent_y, 0)
        return new_point, parent
    
    def get_line_segment(self, x1, y1, x2, y2):
      indices = []
      dx = abs(x2 - x1)
      dy = abs(y2 - y1)
      sx = -1 if x1 > x2 else 1  
      sy = -1 if y1 > y2 else 1 

      if dx > dy:
          err = dx / 2
          while x1 != x2:
              indices.append((x1, y1))
              err -= dy
              if err < 0:
                  y1 += sy
                  err += dx
              x1 += sx
      else:
          err = dy / 2
          while y1 != y2:
              indices.append((x1, y1))
              err -= dx
              if err < 0:
                  x1 += sx
                  err += dy
              y1 += sy
      return indices

    def is_collision(self, node):
      if node.point.x < self.map_x_range[0] or \
         node.point.x > self.map_x_range[1] or \
         node.point.y < self.map_y_range[0] or \
         node.point.y > self.map_y_range[1]:
        return True
      new_point, parent_point = self.coord_to_matrix(node)
      x1 = new_point.x 
      y1 = new_point.y
      x2 = parent_point.x 
      y2 = parent_point.y
      indices = self.get_line_segment(x1, y1, x2, y2)

      for x, y in indices:
        x2 = round(self.env.scale(x, 0, 99, self.map_x_range[0], self.map_x_range[1]), 2)
        y2 = round(self.env.scale(y, 0, 99, self.map_y_range[0], self.map_y_range[1]), 2) 
        self.testing.points.append(Point(x2, y2, 0.3))
        if x > 99 or y > 99:
          return True
        elif math.isnan(self.env.elevation_matrix[x][y]):
          print("nan")
          return True
        elif self.env.elevation_matrix[x][y] > self.limit:
          return True
        # print("height:", round(self.env.elevation_matrix[x][y], 2))
      self.testing_pub.publish(self.testing)
      return False

    def extract_path(self, node_end):
      goal_point = Point(self.goal_node.point.x, self.goal_node.point.y, self.goal_node.point.z)
      goal = Node(goal_point)
      path = [goal]
      node_now = node_end

      while node_now.parent is not None:
        node_now = node_now.parent
        self.marker.points.append(node_now.point)
        new_point = Point(round(node_now.point.x, 1), round(node_now.point.y, 1), round(node_now.point.z, 1))
        new_node = Node(new_point)
        path.insert(0, new_node)

      return path

    def get_distance_and_angle(self, node_start, node_end):
      dx = node_end.point.x - node_start.point.x
      dy = node_end.point.y - node_start.point.y
      return math.hypot(dx, dy), math.atan2(dy, dx)

    def plot_path(self, nodes):
        path_msg = Path()
        path_msg.header.frame_id = self.frame

        for node in nodes:
          pose = PoseStamped()
          pose.header.frame_id = self.frame
          pose.pose.position = Point(node.point.x, node.point.y, 0.05)
          path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        return path_msg
      

def main():
  rrt = RRT(1, 1000)
  path = rrt.planning()
  if path:
    print("Path Found!")
    while not rospy.is_shutdown():
       rrt.plot_path(path)
       rrt.marker_tree_pub.publish(rrt.marker_tree)
       rrt.marker_pub.publish(rrt.marker)
       rrt.marker_dest_pub.publish(rrt.marker_dest)
      
if __name__ == '__main__':
  main()
