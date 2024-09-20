#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

class ClickedPointPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('clicked_point_publisher', anonymous=True)
        
        # Initialize an empty list to store the points
        self.points = []
        
        # Publisher for the point array as a MarkerArray
        self.marker_array_pub = rospy.Publisher('/waypoints', MarkerArray, queue_size=10)
        
        # Subscribe to the /clicked_point topic
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        
        # Timer to publish the MarkerArray at regular intervals
        rospy.Timer(rospy.Duration(1.0), self.publish_marker_array)
        
        rospy.loginfo("ClickedPointPublisher node started. Click on the map in RViz to save and publish points.")
        
    def clicked_point_callback(self, msg):
        # Extract the point data from the message and add it to the list
        point = Point(x=msg.point.x, y=msg.point.y, z=msg.point.z)
        self.points.append(point)
        rospy.loginfo(f"Point added: {point}")

    def publish_marker_array(self, event):
        # Create a MarkerArray message
        marker_array = MarkerArray()
        
        # Create a marker for each point
        for i, point in enumerate(self.points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "clicked_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # Adjust the size of the marker
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Add the marker to the MarkerArray
            marker_array.markers.append(marker)
        
        # Publish the MarkerArray
        self.marker_array_pub.publish(marker_array)
        rospy.loginfo(f"Published {len(self.points)} points as MarkerArray.")

if __name__ == '__main__':
    # Create an instance of the ClickedPointPublisher
    ClickedPointPublisher()
    
    # Spin to keep the script running and listening for callbacks
    rospy.spin()
