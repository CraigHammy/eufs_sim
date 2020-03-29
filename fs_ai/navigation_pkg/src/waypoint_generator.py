#!/usr/bin/env python
import rospy
import math
from perception_pkg.msg import Cone
from geometry_msgs.msg import PoseStamped

cone_messages = []

def generate_waypoints():
    pub = rospy.Publisher('waypoints', PoseStamped, queue_size=10)      # Publish to waypoints topic
    rospy.init_node('waypoints', anonymous=True)                        # Start the waypoints node

    rate = rospy.Rate(10)                                               # Rate is 10hz

    while not rospy.is_shutdown():                                      # Until shutdown / to implement: until final goal is reached (full lap done)
        msg = calculate_midpoint()                                      # TODO: The point between the two cones
        #rospy.loginfo(msg)
        connections = pub.get_num_connections()                         # Wait until connection to publish a message
        if connections > 0:
            pub.publish(msg)
            break
        rate.sleep()


def cone_callback(cone):                                                # Callback to append cones to list
    global cone_messages
    cone_messages.append(cone)


# TODO: Not the fastest implementation! Can use math.hypot or convert to C code to make it faster
# TODO: Add visited cone info
# Can also implement sanity checks like max distance we expect a cone's 'brother' to be
def calculate_midpoint():                                               # Method to calculate the midpoint between a cone and (hopefully) it's brother on the other side
    global cone_messages
    least = 0                                                           # Temp var to use for finding closest cone
    closest_yellow = None                                               # TODO: Handle None type here later...
    closest_blue = None
    midpoint = PoseStamped()

    for c in cone_messages:                                             # Find closest yellow cone to car, Euclidian distance
        if c.colour == "yellow":
            distance = math.sqrt( math.pow(c.location.x, 2) + math.pow(c.location.y, 2) )
            if distance < least:
                least = distance
                closest_yellow = c  # Now have closest yellow cone
    
    least = 0
    for c in cone_messages:                                             # Find closest blue cone to the yellow cone
        if c.colour == "blue":
            distance = math.sqrt( math.pow(closest_yellow.location.x - c.location.x, 2) + \
                math.pow(closest_yellow.location.y - c.location.y, 2) ) # Closest blue cone to yel cone is sqrt( (Xy-Xb)^2 + (Yy-Yb)^2 )
            if distance < least:
                min = distance
                closest_blue = c    # Now have closest blue cone
    
    # Take avg of points and create PoseStamped
    midpoint.header.frame_id = "track"
    midpoint.header.stamp = rospy.Time.now()
    midpoint.pose.position.x = (closest_blue.location.x + closest_yellow.location.x) /2
    midpoint.pose.position.y = (closest_blue.location.x + closest_yellow.location.y) /2
    midpoint.pose.position.z = 0.0
    midpoint.pose.orientation.x = 0.0
    midpoint.pose.orientation.y = 0.0
    midpoint.pose.orientation.z = 0.0
    midpoint.pose.orientation.w = 1.0

    return midpoint # Return the middle point
   

def main():
    global cone_messages                                                # Use the global list of Cones
    sub = rospy.Subscriber("/cones", Cone, cone_callback);              # Create a subscriber

    generate_waypoints()

    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass