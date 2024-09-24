#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray
from pyproj import CRS, Transformer  # Import pyproj for coordinate transformation

class ClickedPointPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('clicked_point_publisher', anonymous=True)
        
        # Initialize an empty list to store the points
        self.waypoints = []
        self.waypoints_latlon = []  # Clear the previous waypoints

        # Initialize UTM origin coordinates (default values)
        self.utm_origin_x = 0.0
        self.utm_origin_y = 0.0
        self.utm_zone = int()  # Replace with your specific UTM zone
        self.utm_origin_received = False
        


        # Publisher for the point array as a MarkerArray
        self.waypoints_array_pub = rospy.Publisher('/waypoints', MarkerArray, queue_size=10)
        # Publisher for latitude and longitude
        self.waypoints_latlon_array_pub = rospy.Publisher('/waypoints_latlon', Float64MultiArray, queue_size=10)
        
        
        # Subscribe to the /clicked_point topic
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        # Subscribe to the /utm_coordinates topic
        rospy.Subscriber('/utm_coordinates', Float64MultiArray, self.utm_coordinates_callback)
        
        # Timer to publish the MarkerArray and lat/lon waypoints at regular intervals
        rospy.Timer(rospy.Duration(1.0), self.publish_waypoints_array)
        rospy.Timer(rospy.Duration(1.0), self.publish_waypoints_latlon_array)
        
        
        rospy.loginfo("ClickedPointPublisher node started. Click on the map in RViz to save and publish points.")


    def utm_coordinates_callback(self, msg):
        # Update the UTM origin coordinates
        self.utm_origin_x = msg.data[0]
        self.utm_origin_y = msg.data[1]
        self.utm_zone = int(msg.data[2])


        # Initialize the transformer for UTM to WGS84 conversion
        self.utm_crs = CRS.from_proj4(f"+proj=utm +zone={self.utm_zone} +datum=WGS84 +units=m +no_defs")
        self.wgs84_crs = CRS.from_epsg(4326)  # WGS84 Latitude/Longitude
        self.transformer_to_wgs84 = Transformer.from_crs(self.utm_crs, self.wgs84_crs, always_xy=True)

        self.utm_origin_received = True
        #rospy.loginfo(f"Received UTM origin coordinates: x={self.utm_origin_x}, y={self.utm_origin_y}, utm_zone={self.utm_zone}")


    def clicked_point_callback(self, msg):
        if not self.utm_origin_received:
            rospy.logwarn("UTM origin coordinates not received yet. Cannot add point.")
            return
        
        # Extract the point data from the message and add it to the list
        waypoint = Point(x=msg.point.x, y=msg.point.y, z=msg.point.z)
        self.waypoints.append(waypoint)
        rospy.loginfo(f"Point added: {waypoint}")

        # Convert the clicked point in the local frame to the global UTM frame using UTM origin
        global_x = self.utm_origin_x + msg.point.x
        global_y = self.utm_origin_y + msg.point.y

        # Convert global UTM coordinates to WGS84 latitude and longitude
        lon, lat = self.transformer_to_wgs84.transform(global_x, global_y)

        waypoint_latlon = [lat, lon]  # Store as a list [latitude, longitude]
        self.waypoints_latlon.append(waypoint_latlon)

        rospy.loginfo(f"Point added: Local (X: {msg.point.x}, Y: {msg.point.y}), Latitude/Longitude: (Lat: {lat}, Lon: {lon})")
        

    def publish_waypoints_array(self, event):
        # Create a MarkerArray message
        marker_array = MarkerArray()
        
        # Create a marker for each point
        for i, point in enumerate(self.waypoints):
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
        self.waypoints_array_pub.publish(marker_array)
        rospy.loginfo(f"Published {len(self.waypoints)} points as MarkerArray.")


    def publish_waypoints_latlon_array(self, event):
        # Create a Float64MultiArray message
        waypoints_latlon_array = Float64MultiArray()
        
        # Flatten the list of lat/lon waypoints and append to data
        for waypoint_latlon in self.waypoints_latlon:
            waypoints_latlon_array.data.extend(waypoint_latlon)  # Add each [lat, lon] to the array

        # Publish the Float64MultiArray containing all waypoints
        self.waypoints_latlon_array_pub.publish(waypoints_latlon_array)
        rospy.loginfo(f"Published {len(self.waypoints_latlon)} waypoints as Latitude/Longitude array.")


if __name__ == '__main__':
    # Create an instance of the ClickedPointPublisher
    ClickedPointPublisher()
    
    # Spin to keep the script running and listening for callbacks
    rospy.spin()
