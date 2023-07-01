#!/usr/bin/env python3

import sys 

package_dir = '/home/aleinin/rrt_ws/src/rrt_package'
sys.path.append(package_dir)

import rospy

import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Bool
from tf.transformations import quaternion_from_euler

from rrt_package.srv import FollowPath, FollowPathResponse

class FollowPathServer:
  def __init__(self):
    rospy.init_node('FollowPathServer')
    rospy.loginfo('FollowPathServer started...')
    rospy.Service('follow_path_service', FollowPath, self.handle_follow_path)

    self.path = None
    self.path_length = 0
    self.goal_reached = False
    self.i = 1
    
    self.driver = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

  def handle_follow_path(self, req):
    received_path = req.path

    self.process_path(received_path)
    # self.distance_to_goal = rospy.Subscriber('/diff_drive_go_to_goal/distance_to_goal', Float32, callback=self.distance_to_goal_callback)
    self.goal_achieved = rospy.Subscriber('goal_achieved', Bool, callback=self.goal_achieved_callback)

    while not self.goal_reached:
      # rospy.wait_for_message('/diff_drive_go_to_goal/distance_to_goal', Float32)
      rospy.wait_for_message('goal_achieved', Bool)

    response = FollowPathResponse()

    # rospy.loginfo('FollowPath: distance_to_goal unregistered...')
    # self.distance_to_goal.unregister()
    self.goal_achieved.unregister()
    reached = True
    self.path = None
    self.path_length = 0
    self.goal_reached = False
    self.i = 1

    response.achieved = Bool(reached)
    return response

  def generate_quaternion(self):
    for i in range(self.path_length - 1):
      x1 = self.path.poses[i].pose.position.x
      y1 = self.path.poses[i].pose.position.y
      x2 = self.path.poses[i + 1].pose.position.x
      y2 = self.path.poses[i + 1].pose.position.y
      yaw = math.atan2((y2 - y1), (x2 - x1))
      q = quaternion_from_euler(0, 0, yaw)
      self.path.poses[i + 1].pose.orientation.x = q[0]
      self.path.poses[i + 1].pose.orientation.y = q[1]
      self.path.poses[i + 1].pose.orientation.z = q[2]
      self.path.poses[i + 1].pose.orientation.w = q[3]

  def process_path(self, path):
    self.path = path 
    for _ in path.poses:
      self.path_length += 1
    rospy.loginfo('Path of length ' + str(self.path_length - 1) + ' received')
    self.generate_quaternion()
    self.publish_next_goal()
  
  def publish_next_goal(self):
    goal = self.path.poses[self.i]
    point = 'Point(' + str(goal.pose.position.x) + ", " + str(goal.pose.position.y) + ") "
    rospy.loginfo(point + str(self.i) + ' of ' + str(self.path_length - 1))
    self.driver.publish(goal)
    self.i += 1

  def distance_to_goal_callback(self, dist):
    if dist.data <= Float32(0.2).data:
      if self.i > (self.path_length - 1):
        self.goal_reached = True
        return 
      self.publish_next_goal()

  def goal_achieved_callback(self, achieved):
    if achieved.data:
      if self.i > (self.path_length - 1):
        rospy.loginfo('FollowPathServer: goal_achieved unregistered...')
        self.goal_reached = True
        return 
      self.publish_next_goal()
  
  def spin(self):
    rospy.spin()
    
if __name__ == '__main__':
  # Initialize FollowPath Server
  fpServer = FollowPathServer()
  fpServer.spin()