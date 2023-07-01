import rospy

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

elevation_topic = '/elevation_map_raw_visualization/elevation_cloud'
map_region_topic = '/elevation_map_raw_visualization/map_region'

def map_region_callback(msg):
  print("Spinning raw map region")
  
def elevation_points_callback(msg):
  print("Spinning raw elevation points")

if __name__ == '__main__':
  rospy.init_node('SPINNING')
  elevation_map_region_sub = rospy.Subscriber(map_region_topic, Marker, map_region_callback)
  elevation_map_subscriber = rospy.Subscriber(elevation_topic, PointCloud2, elevation_points_callback)  
  rospy.spin()