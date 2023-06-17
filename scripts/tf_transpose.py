import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
from visualization_msgs.msg import Marker

import tf
from tf.transformations import *
import numpy as np
import tf2_ros

class tf_transpose:
  def __init__(self):
    self.init = rospy.init_node("tf_transpose")
    self.frame = "world"

    self.listener = tf.TransformListener()
    self.listener.waitForTransform('/base_link', '/world', rospy.Time(0), rospy.Duration(4.0))

    self.source_frame = 'base_link'
    self.target_frame = 'world'

    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    self.transform = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time(0), rospy.Duration(1.0))

    self.pose_path_pub = rospy.Publisher("/world/path", Path, queue_size=10)

    self.marker_nodes_pub = rospy.Publisher('/world/nodes', Marker, queue_size=10)
    self.marker_nodes = Marker()
    self.marker_nodes.header.frame_id = self.frame
    self.marker_nodes.type = Marker.POINTS
    self.marker_nodes.action = Marker.ADD 
    self.marker_nodes.pose.orientation.w = 1.0
    self.marker_nodes.scale.x = 0.1
    self.marker_nodes.scale.y = 0.1
    self.marker_nodes.color.a = 1.0
    self.marker_nodes.color.g = 1.0

    self.marker_tree_pub = rospy.Publisher('/world/tree', Marker, queue_size=10)
    self.marker_tree = Marker()
    self.marker_tree.header.frame_id = self.frame 
    self.marker_tree.type = Marker.LINE_LIST
    self.marker_tree.action = Marker.ADD 
    self.marker_tree.pose.orientation.w = 1.0
    self.marker_tree.scale.x = 0.01
    self.marker_tree.scale.y = 0.01
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

    self.path_data = None
    self.node_data = None
    self.dest_data = None
    self.tree_data = None

    self.marker_path_sub = rospy.Subscriber('/base_link/path', Path, callback=self.path_callback)
    self.marker_nodes_sub = rospy.Subscriber('/base_link/nodes', Marker, callback=self.nodes_callback)
    self.marker_dest_sub = rospy.Subscriber('/base_link/dest', Marker, callback=self.dest_callback)
    self.marker_tree_sub = rospy.Subscriber('/base_link/tree', Marker, callback=self.tree_callback)
    # rospy.wait_for_message('/base_link/tree', Marker)

  def convert(self, point):
    translation = self.transform.transform.translation
    rotation = self.transform.transform.rotation

    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = tf.transformations.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])[:3, :3]
    transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]

    p_r = [point.x, point.y, point.z, 1]
    p_w = transform_matrix@p_r
    return Point(round(p_w[0], 1), round(p_w[1], 1), round(p_w[2], 1))

  def path_callback(self, data):
    self.path_data = data
  
  def path_pub(self):
    if self.path_data is not None:
      path_msg = Path()
      path_msg.header.frame_id = self.target_frame

      for ps in self.path_data.poses:
        converted_point = self.convert(ps.pose.position)
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.pose.position = converted_point
        path_msg.poses.append(pose)

      self.pose_path_pub.publish(path_msg)

  def nodes_callback(self, data):
    self.node_data = data
  
  def nodes_pub(self):
    if self.node_data is not None:
        for point in self.node_data.points:
            self.marker_nodes.points.append(self.convert(point))
        self.marker_nodes_pub.publish(self.marker_nodes)
        self.marker_nodes.points = []
      
  def dest_callback(self, data):
    self.dest_data = data

  def dest_pub(self):
    if self.dest_data is not None:
      for point in self.dest_data.points:
        self.marker_dest.points.append(self.convert(point))
      self.marker_dest_pub.publish(self.marker_dest)
      self.marker_dest.points = []

  def tree_callback(self, data):
    self.tree_data = data

  def tree_pub(self):
    if self.tree_data is not None:
      for i, j in zip(self.tree_data.points[::2], self.tree_data.points[1::2]):
        self.marker_tree.points.append(self.convert(i))
        self.marker_tree.points.append(self.convert(j))
      self.marker_tree_pub.publish(self.marker_tree)
      self.marker_tree.points = []  

if __name__ == '__main__':
  t = tf_transpose()
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    t.path_pub()
    t.dest_pub()
    t.tree_pub()
    t.nodes_pub()
    rate.sleep()