#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseStamped
from perception_pkg.msg import Cone
from visualization_msgs.msg import Marker

class WaypointGenerator:

    def __init__(self):
        self.cone_messages = []
        self.sub = rospy.Subscriber("/cones", Cone, self.cone_callback);            # Create a subscriber
        self.pub = rospy.Publisher('/waypoints', PoseStamped, queue_size=1)         # Publish to waypoints topic
        self.markerPub = rospy.Publisher('/waypoints_markers', Marker)

    def generate_waypoints(self):
        rate = rospy.Rate(5)                                                        # Rate is 5hz

        while not rospy.is_shutdown():                                              # Until shutdown / to implement: until final goal is reached (full lap done)
            msg = self.calculate_midpoint()                                         # TODO: The point between the two cones
            #rospy.loginfo(msg)
            rate.sleep()


    def cone_callback(self, cone):                                                  # Callback to append cones to list
        self.cone_messages.append(cone)
        #print(cone_messages)


    # TODO: Not the fastest implementation! Can use math.hypot or convert to C code to make it faster
    # TODO: Add visited cone info
    # Can also implement sanity checks like max distance we expect a cone's 'brother' to be
    def calculate_midpoint(self):                                               # Method to calculate the midpoint between a cone and (hopefully) it's brother on the other side  
        least = 999999                                                          # Temp var to use for finding closest cone
        closest_yel = Cone()                                                    # TODO: Handle None type here later...
        closest_blu = Cone()
        yel_created = False
        blu_created = False
        midpoint = PoseStamped()

        for c in self.cone_messages:                                             # Find closest yellow cone to car, Euclidian distance
            #print(c)
            if c.colour == "yellow":
                distance = math.sqrt( math.pow(c.location.x, 2) + math.pow(c.location.y, 2) )
                if distance < least:
                    least = distance
                    closest_yel = c                 # Now have closest yellow cone
                    yel_created = True
                    #print("------CLOSEST YEL-----")
                    #print(closest_yel)
                    #print("------CLOSEST YEL-----")
            least = 99999

        for c in self.cone_messages:                                             # Find closest blue cone to the yellow cone
            if c.colour == "blue":
                distance = math.sqrt( math.pow(closest_yel.location.x - c.location.x, 2) + \
                    math.pow(closest_yel.location.y - c.location.y, 2) ) # Closest blue cone to yel cone is sqrt( (Xy-Xb)^2 + (Yy-Yb)^2 )
                if distance < least:
                    min = distance
                    closest_blu = c                 # Now have closest blue cone
                    blu_created = True
                    #print("------CLOSEST BLU-----")
                    #print(closest_blu)
                    #print("------CLOSEST BLU-----")
        
        if yel_created is True and blu_created is True:      # Do nothing if they cones arent established / Take avg of points and create PoseStamped / check if its just a default cone and do nothing if it is
            
            midpoint.header.frame_id = "track"
            midpoint.header.stamp = rospy.Time.now()
            midpoint.pose.position.x = (closest_blu.location.x + closest_yel.location.x) /2
            midpoint.pose.position.y = (closest_blu.location.x + closest_yel.location.y) /2
            midpoint.pose.position.z = 0.0
            midpoint.pose.orientation.x = 0.0
            midpoint.pose.orientation.y = 0.0
            midpoint.pose.orientation.z = 0.0
            midpoint.pose.orientation.w = 1.0

            # Create a marker to visualize the midpoint in RViz
            marker = Marker()
            marker.header.frame_id = "/track"
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = midpoint.pose.position.x
            marker.pose.position.y = midpoint.pose.position.y
            marker.pose.position.z = 0.0

            connections = self.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(midpoint)
                self.markerPub.publish(marker)
   
    
if __name__ == '__main__':
    rospy.init_node('waypoints', anonymous=True)            # Start the waypoints node
    w = WaypointGenerator()                                 # Init object (creates publisher, subscriber and list of cones)

    while not rospy.is_shutdown():                          # Start generating waypoints between cones
        w.generate_waypoints();

    rospy.spin()