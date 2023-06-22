#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from integration import RRT 
from follow_path import FollowPath

class navigation:
    def __init__(self, goal):
      rospy.init_node('Navigation')
      self.rrt = RRT(0.1, 1000, 0.01, goal)
      self.follow = FollowPath()
    
    def autonomous(self):
      count = 0
      while not self.rrt.goal_found:
        rospy.loginfo('RRT sampling: ' + str(count))
        path = self.rrt.planning()
        # follow(path)


def main():
  goal = Point(5, -5, 0)
  nav = navigation(goal)
  nav.autonomous()

if __name__ == '__main__':
  main()