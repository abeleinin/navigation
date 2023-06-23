#!/usr/bin/env python3

from threading import Thread
import rospy

from geometry_msgs.msg import Point
from integration import RRT 
from follow_path import FollowPath

class navigation:
    def __init__(self, goal):
      # rospy.init_node('Navigation')
      self.followPath = None
      self.curr_rrt = None
      self.goal = goal
      self.path = None
      self.follow_path_thread = Thread(target=self.run_follow_path)
      self.path_pub_thread = Thread(target=self.run_pub_path)
    
    def autonomous(self):
      count = 0
      self.curr_rrt = RRT(1, 50, 0.01, self.goal)
      while not self.curr_rrt.goal_found:
        self.follow_path_thread.start()
        rospy.loginfo('RRT sampling: ' + str(count))
        self.path = self.rrt.planning()
        self.path_pub_thread.start()
        self.follow_path_thread.join()
        count += 1
        self.curr_rrt = RRT(1, 50, 0.01, self.goal)
      rospy.loginfo('Path Completed')
    
    def run_follow_path(self):
      self.followPath = FollowPath()
    
    def run_pub_path(self):
      self.curr_rrt.plot_path(self.path)
      while not rospy.is_shutdown():
        self.rrt.path_pub.publish(self.curr_rrt.curr_path_msg)


def main():
  goal = Point(7, -5, 0)
  nav = navigation(goal)
  nav.autonomous()

if __name__ == '__main__':
  main()