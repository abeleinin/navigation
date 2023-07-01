#!/usr/bin/env python3

import sys 

package_dir = '/home/aleinin/rrt_ws/src/rrt_package'
scripts = '/home/aleinin/rrt_ws/src/rrt_package/scripts'
sys.path.append(package_dir)
sys.path.append(scripts)

import math
import random
import rospy

from nav_msgs.msg import Path  
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PoseStamped, Vector3
from visualization_msgs.msg import Marker

from process_map import elevationMap
from robot_odom import robotOdom
from rrt_package.srv import FollowPath, FollowPathRequest 

from followPathServer import FollowPathServer

class Node:
    def __init__(self, point: Point):
      self.point = point 
      self.angle = 0
      self.parent = None

class FollowPathClient:
    def __init__(self, step_len, iter_max, elev_limit, goal):
      rospy.init_node('FollowPathClient')
      rospy.loginfo('FollowPathClient started...')
      rospy.wait_for_service('follow_path_service')

      # retrieve environmental and robot information

      ### parameters ###
      self.step_len = step_len
      self.iter_max = iter_max
      self.limit = elev_limit
      self.goal_point = goal
      self.goal_found = False

      ### global variables ###
      self.init()

      self.follow_path = rospy.ServiceProxy('follow_path_service', FollowPath)

      ### publishers ###
      self.frame = 'world'
      self.path_pub = rospy.Publisher('/world/path', Path, queue_size=10)

      # All nodes
      self.marker_pub = rospy.Publisher('/world/nodes', Marker, queue_size=10)
      self.marker = Marker()
      self.marker.header.frame_id = self.frame
      self.marker.type = Marker.POINTS
      self.marker.action = Marker.ADD 
      self.marker.pose.orientation.w = 1.0
      self.marker.scale = Vector3(0.1, 0.1, 0)
      self.marker.color = ColorRGBA(0, 1, 0, 1) # red
      self.marker.points.append(self.start_point)

      # rrt tree 
      self.marker_tree_pub = rospy.Publisher('/world/tree', Marker, queue_size=10)
      self.marker_tree = Marker()
      self.marker_tree.header.frame_id = self.frame
      self.marker_tree.type = Marker.LINE_LIST
      self.marker_tree.action = Marker.ADD 
      self.marker_tree.pose.orientation.w = 1.0
      self.marker_tree.scale = Vector3(0.03, 0.03, 0)
      self.marker_tree.color = ColorRGBA(1, 0.6, 0, 1) # orange

      # start and goal nodes (Large Yellow S)
      self.marker_dest_pub = rospy.Publisher('/world/dest', Marker, queue_size=10)
      self.marker_dest = Marker()
      self.marker_dest.header.frame_id = self.frame
      self.marker_dest.type = Marker.POINTS
      self.marker_dest.action = Marker.ADD 
      self.marker_dest.pose.orientation.w = 1.0
      self.marker_dest.scale = Vector3(0.3, 0.3, 0)
      self.marker_dest.color = ColorRGBA(1, 1, 0, 1) # yellow
      self.marker_dest.points.append(self.start_point)
      self.marker_dest.points.append(self.goal_point)

      # bresenham points created during branch collision checking
      self.bresenham_pub = rospy.Publisher('/world/bresenham', Marker, queue_size=10)
      self.bresenham = Marker()
      self.bresenham.header.frame_id = self.frame
      self.bresenham.type = Marker.POINTS
      self.bresenham.action = Marker.ADD 
      self.bresenham.pose.orientation.w = 1.0
      self.bresenham.scale = Vector3(0.1, 0.1, 0)
      self.bresenham.color = ColorRGBA(0, 0, 1, 1) # blue

    def init(self):
      rospy.loginfo("FollowPathClient: Init params...")
      self.map = elevationMap()
      self.jackal = robotOdom()
      self.start_point = self.jackal.position 
      print(self.start_point)
      self.start_node = Node(self.start_point)
      self.goal_node = Node(self.goal_point)
      self.front_parent = self.start_node
      self.back_parent = self.start_node
      self.front_leaves = []
      self.back_leaves = []
      self.path_msg = Path()
     
    def planning(self):
      for i in range(self.iter_max):
        if i % 10 == 0:
          rospy.loginfo('Planning iteration: ' + str(i))

        # find path in front or behind robot
        if self.front_parent != None:
          front_path = self.generate_leafs(3, 0 + self.jackal.yaw, self.front_parent, self.front_leaves)
        if self.back_parent != None:
          back_path = self.generate_leafs(3, 180 + self.jackal.yaw, self.back_parent, self.back_leaves)

        # check if a path was found
        if front_path != None:
          self.goal_found = True
          return front_path
        elif back_path != None:
          self.goal_found = True
          return back_path

        # random leaf selection 
        # determine the next parent node. if leaves list is empty then all leaves lead to a collision.
        self.front_parent = self.front_leaves.pop(random.randint(0, len(self.front_leaves) - 1)) if self.front_leaves else None
        self.back_parent = self.back_leaves.pop(random.randint(0, len(self.back_leaves) - 1)) if self.back_leaves else None

        # sequential leaf selection
        # self.front_parent = self.front_leaves.pop(0) if self.front_leaves else None
        # self.back_parent = self.back_leaves.pop(0) if self.back_leaves else None

      # return path nearest to node
      leaves = self.front_leaves + self.back_leaves
      nearest = self.nearest_to_goal(leaves)
      return self.extract_path(nearest)


    def forward_planning(self):
      for i in range(self.iter_max):
        if i % 10 == 0:
          rospy.loginfo('Planning iteration: ' + str(i))

        # find path in front or behind robot
        if self.front_parent != None:
          front_path = self.generate_leafs(3, 0 + self.jackal.yaw, self.front_parent, self.front_leaves)

        # check if a path was found
        if front_path != None:
          self.goal_found = True
          return front_path

        # random leaf selection 
        # determine the next parent node. if leaves list is empty then all leaves lead to a collision.
        self.front_parent = self.front_leaves.pop(random.randint(0, len(self.front_leaves) - 1)) if self.front_leaves else None

      # return path nearest to node
      leaves = self.front_leaves + self.back_leaves
      nearest = self.nearest_to_goal(leaves)
      return self.extract_path(nearest)   

    def generate_leafs(self, branching, orientation, curr_parent, leaves):
      # generate random angles separated by delta
      delta = 15
      angles = [] 
      while len(angles) < branching:
        new_angle = random.randint(-45, 45)
        if all(abs(new_angle - existing_angle) >= delta for existing_angle in angles):
          angles.append(new_angle)
      # set angels
      # angles = [-60, -45, 0, 45, 60] 
      for angle in angles:
        # calculate new Point(x, y) values for a branch of length step_len at the specified angle using trig 
        x = curr_parent.point.x + self.step_len * math.cos(math.radians(angle + curr_parent.angle + orientation))
        y = curr_parent.point.y + self.step_len * math.sin(math.radians(angle + curr_parent.angle + orientation))
        new_node = Node(Point(x, y, 0.05))
        new_node.parent = curr_parent
        new_node.angle = angle + curr_parent.angle
        
        # check for collision between the new line segment created and the elevation map
        collision = self.is_collision(new_node)
        if collision:
          continue

        # add to visualization arrays 
        self.marker_tree.points.append(Point(new_node.point.x, new_node.point.y, 0.05))
        self.marker_tree.points.append(Point(new_node.parent.point.x, new_node.parent.point.y, 0.05))
        self.marker_tree_pub.publish(self.marker_tree)
        self.marker_dest_pub.publish(self.marker_dest)
        # sleep for visualization purposes
        rospy.sleep(0.01)

        # calculate euclidean distance from the new node to the goal
        dist, _ = self.get_distance_and_angle(new_node, self.goal_node)
        if dist <= self.step_len:
          dist, theta = self.get_distance_and_angle(new_node, self.goal_node)

          dist = min(self.step_len, dist)
          # create final node
          final_point = Point(new_node.point.x + dist * math.cos(theta), 
                              new_node.point.y + dist * math.sin(theta), 0)
          # ??? isn't final_node just the goal node
          # ??? new_node.parent might be more correct (test)
          final_node = Node(final_point)
          final_node.parent = new_node
          # add to visualization arrays
          self.marker_tree.points.append(new_node.parent.point)
          self.marker_tree.points.append(final_node.point)
          # return the list of nodes to goal found by recursing through each nodes parent
          return self.extract_path(new_node)
        else: 
          # appends to list of all possible leaf nodes
          leaves.append(new_node)
      return None

    def coord_to_matrix(self, node):
        x = round(self.map.scale(node.point.x, self.map.x_min, self.map.x_max, 0, self.map.x_range - 1))
        y = round(self.map.scale(node.point.y, self.map.y_min, self.map.y_max, 0, self.map.x_range - 1))
        parent_x = round(self.map.scale(node.parent.point.x, self.map.x_min, self.map.x_max, 0, self.map.x_range - 1))
        parent_y = round(self.map.scale(node.parent.point.y, self.map.y_min, self.map.y_max, 0, self.map.x_range - 1))
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
      if node.point.x < self.map.x_min or \
         node.point.x > self.map.x_max or \
         node.point.y < self.map.y_min or \
         node.point.y > self.map.y_max:
        return True
      new_point, parent_point = self.coord_to_matrix(node)
      x1 = new_point.x 
      y1 = new_point.y
      x2 = parent_point.x 
      y2 = parent_point.y
      indices = self.get_line_segment(x1, y1, x2, y2)

      for x, y in indices:
        x2 = round(self.map.scale(x, 0, self.map.x_range - 1, self.map.x_min, self.map.x_max), 2)
        y2 = round(self.map.scale(y, 0, self.map.x_range - 1, self.map.y_min, self.map.y_max), 2) 
        self.bresenham.points.append(Point(x2, y2, 0.3))
        if x > self.map.x_range - 1 or y > self.map.x_range - 1:
          return True
        # elif math.isnan(self.map.elevation_matrix[x][y]):
        #   print('nan')
        #   return True
        elif self.map.elevation_matrix[x][y] > self.limit:
          return True
        # print('height:', round(self.map.elevation_matrix[x][y], 2))
      self.bresenham_pub.publish(self.bresenham)
      return False

    # def extract_path(self, node_end):
    #   goal_point = Point(self.goal_node.point.x, self.goal_node.point.y, self.goal_node.point.z)
    #   goal = Node(goal_point)
    #   path = [goal]
    #   node_now = node_end

    #   while node_now.parent is not None:
    #     node_now = node_now.parent
    #     self.marker.points.append(node_now.point)
    #     new_point = Point(round(node_now.point.x, 1), round(node_now.point.y, 1), round(node_now.point.z, 1))
    #     new_node = Node(new_point)
    #     path.insert(0, new_node)

    #   return path

    def extract_path(self, curr_node):
      path = [self.start_node]

      while curr_node.parent is not None:
        self.marker.points.append(curr_node.point)
        curr_point = Point(round(curr_node.point.x, 1), round(curr_node.point.y, 1), round(curr_node.point.z, 1))
        waypoint = Node(curr_point)
        path.insert(1, waypoint)
        curr_node = curr_node.parent

      return path

    def get_distance_and_angle(self, node_start, node_end):
      dx = node_end.point.x - node_start.point.x
      dy = node_end.point.y - node_start.point.y
      return math.hypot(dx, dy), math.atan2(dy, dx)
    
    # find nearest leaf node to the goal node
    def nearest_to_goal(self, leaves):
      nearest_dist = float('inf')
      nearest_node = None
      for node in leaves:
        curr_dist, _ = self.get_distance_and_angle(node, self.goal_node)
        if curr_dist < nearest_dist:
          nearest_dist = curr_dist
          nearest_node = node
      return nearest_node

    def spin_markers(self):
      while not rospy.is_shutdown():
        self.marker_tree_pub.publish(self.marker_tree)
        self.marker_pub.publish(self.marker)
        self.marker_dest_pub.puublish(self.marker_dest)

    def plot_path(self, nodes):
      self.path_msg.header.frame_id = self.frame

      for node in nodes:
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.pose.position = Point(node.point.x, node.point.y, 0.05)
        self.path_msg.poses.append(pose)
        
      self.path_pub.publish(self.path_msg)
      return self.path_msg
    
    def handle_follow_path(self):
      try:
        nodes = self.planning()
        if nodes:
          rospy.loginfo('Path Found!')
          path = self.plot_path(nodes)

          req = FollowPathRequest()
          req.path = path
          
          print("Waiting for response")
          response = self.follow_path(req)
          print(response)

          while not self.goal_found:
            rospy.sleep(2)
            self.init()
            nodes = self.forward_planning()
            req.path = self.plot_path(nodes)
            response = self.follow_path(req)
          
          print("CLIENT: Goal Achieved")

            # nodes = self.planning()
            # path = self.plot_path(nodes)
            # req.path = path

      except rospy.ServiceException as e:
        rospy.logerr("Service call failed:", e)


def main():
  # Initialize FollowPath Client
  step_len = 1
  iter_max = 50
  limit = 0.01
  goal = Point(7, -3, 0)

  fpClient = FollowPathClient(step_len, iter_max, limit, goal)
  fpClient.handle_follow_path()
      
if __name__ == '__main__':
  main()
