#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import yaml

def save_gps_origin(gps_data):
    # Save the GPS coordinates to a dictionary
    gps_origin = {
        'latitude': gps_data.latitude,
        'longitude': gps_data.longitude,
        'altitude': gps_data.altitude,
        'datum': [gps_data.latitude, gps_data.longitude, gps_data.altitude, 'map', 'base_link']
    }
    
    # Save the data to a YAML file
    with open('/home/kimkt0408/catkin_ws/src/gtsam_test/gps_data/map_origin_gps.yaml', 'w') as file:
        yaml.dump(gps_origin, file)

    rospy.loginfo("GPS data of the map origin saved: %s", gps_origin)
    rospy.signal_shutdown("GPS data of the map origin saved.")

if __name__ == "__main__":
    rospy.init_node('map_gps_origin_saver')

    # Subscribe to the GPS topic
    rospy.Subscriber("/gps/fix", NavSatFix, save_gps_origin)

    rospy.loginfo("Waiting for GPS data...")
    rospy.spin()
