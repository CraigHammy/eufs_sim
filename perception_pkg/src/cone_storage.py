#! /usr/bin/env python

import rospy
import tf
import os
import time
import random
from math import fabs, pi, sqrt, pow
from perception_pkg.msg import Cone
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import PoseStamped, Twist, Pose, Vector3, PointStamped, Point32, Point, PolygonStamped, Transform
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

signs = MarkerArray()
def euclidean_distance(pose1, pose2):
        return sqrt(pow(pose2.position.x - pose1.position.x, 2) + pow(pose2.position.y - pose1.position.y, 2))

def callback(msg):
    print "callback"
    try:
        (trans,_) =transform_listener.lookupTransform('odom', msg.child_frame_id, rospy.Time(0))
        state = 0
        transPose =Pose()
        
        print trans
        print msg.location
        transPose.position.x=trans[0] + msg.location.x
        transPose.position.y=trans[1] + msg.location.y
        print transPose
        for i in range(len(signs.markers)):
            if(euclidean_distance(signs.markers[i].pose, transPose)<3):
                state = 1


        if not state:
         marker =Marker()

         marker.header.frame_id="odom"
         marker.header.stamp=rospy.Time.now()
         marker.ns = "stored cones"
         marker.id = len(signs.markers) -1
         marker.type = Marker.CYLINDER
         marker.action = Marker.ADD
         marker.pose.position.x = transPose.position.x
         marker.pose.position.y = transPose.position.y
         marker.scale.x = 0.1
         marker.scale.y = 0.1
         marker.scale.z = 0.1
         marker.color.a = 1.0

         if(msg.colour == "blue"):
             marker.color.b=1.0
         else:
            marker.color.r =1.0
            marker.color.g = 1.0

         marker.color.r = 1.0
         signs.markers.append(marker)

    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print "Not dealing with this shite"

    pub.publish(signs)


rospy.init_node("globalconemarkers")
transform_listener = tf.TransformListener()

sub = rospy.Subscriber("/cones", Cone, callback)
pub = rospy.Publisher("/conelocs",MarkerArray, queue_size=1,)
rospy.spin()