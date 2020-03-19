#! /usr/bin/env python

import rospy
import pymap3d as pm
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geodetic_to_enu_conversion_pkg.msg import Gps

def gps_callback(msg):
    global datum_lat
    global datum_lon
    global i
    if i==0:
        datum_lat = msg.latitude
        datum_lon = msg.longitude
        i = i + 1

    current_lat = msg.latitude
    current_lon = msg.longitude
    x, y, _ = pm.geodetic2enu(current_lat, current_lon, 0, datum_lat, datum_lon, 0)
    position = Gps()
    position.x = x
    position.y = y
    pub.publish(position)

if __name__ == '__main__':
    global i
    i = 0
    
    rospy.init_node("geodetic_to_enu_conversion_node")
    sub = rospy.Subscriber("/gps", NavSatFix, gps_callback)
    pub = rospy.Publisher("/odometry/gps", Gps, queue_size=1)
    rospy.spin()