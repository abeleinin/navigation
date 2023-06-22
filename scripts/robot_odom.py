import math

import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class robotOdom:
  def __init__(self):
    self.position = None
    self.orientation = None
    self.yaw = None
    self.robot_odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.robot_odom_sub)
    rospy.wait_for_message('/ground_truth/state', Odometry)

  def robot_odom_sub(self, odom):
    self.position = odom.pose.pose.position 
    self.position.x = round(self.position.x, 2)
    self.position.y = round(self.position.y, 2)
    self.position.z = round(self.position.z, 2)
    self.orientation = odom.pose.pose.orientation
    quaternion = [self.orientation.x,
                  self.orientation.y,
                  self.orientation.z,
                  self.orientation.w]
    euler = tft.euler_from_quaternion(quaternion)
    self.yaw = round(math.degrees(euler[2]))

if __name__ == '__main__':
  rospy.init_node('robotPose')
  robot = robotOdom()
  print(robot.position)
  print(robot.yaw)