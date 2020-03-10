#! /usr/bin/python

#Taken from https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f#file-ros_image_saver-py and modified for our use

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import os

# Instantiate CvBridge
bridge = CvBridge()
image_folder_path = '/home/craig/ros_space/masters_catkin/src/eufs_sim/fs_ai/yolo_files/darknet_ros/darknet/custom_data/images'

def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        i = len(os.listdir(image_folder_path))
        print i 
        image_name = 'rightcamera' + str(i) +'.jpg'
        cv2.imwrite(image_name, cv2_img)
        rospy.sleep(1)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    image_topic = "/zed/right/image_rect_color"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()