#!/usr/bin/env python3

import rospy

import numpy as np
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler

class FollowPath:
  def __init__(self):
    rospy.loginfo('FollowPath: Node started...')

    self.path = None
    self.path_length = 0
    self.goal_reached = False
    self.i = 1

    self.driver = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

    self.path_sub = rospy.Subscriber('world/path', Path, callback=self.path_callback)
    self.distance_to_goal = rospy.Subscriber('/diff_drive_go_to_goal/distance_to_goal', Float32, callback=self.distance_to_goal_callback)

    rospy.wait_for_message('world/path', Path)
    while not self.goal_reached:
      rospy.wait_for_message('/diff_drive_go_to_goal/distance_to_goal', Float32)

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

  # def run(self):
  #   while not rospy.is_shutdown() and self.i < (self.path_length - 1):
  #     if self.goal_reached:
  #       rospy.signal_shutdown()
  #     rospy.spin()

  def path_callback(self, path):
    rospy.loginfo('Path received')
    self.path = path 
    for _ in path.poses:
      self.path_length += 1
    self.generate_quaternion()
    self.publish_next_goal()
    self.path_sub.unregister()
  
  def publish_next_goal(self):
    rospy.loginfo('Going to point: ' + str(self.i))
    self.driver.publish(self.path.poses[self.i])
    self.i += 1

  def distance_to_goal_callback(self, dist):
    if dist.data <= Float32(0.2).data:
      if self.i > (self.path_length - 1):
        self.goal_reached = True
        rospy.loginfo('FollowPath: distance_to_goal unregistered...')
        self.distance_to_goal.unregister()
        return 
      self.publish_next_goal()

      if self.i >= self.path_length:
        self.goal_reached = True
    
if __name__ == '__main__':
  rospy.init_node('driver')
  follow = FollowPath()
  # follow.run()