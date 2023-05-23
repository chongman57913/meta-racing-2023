#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import time
import sys

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    filename = 'output_{}.jpg'.format(int(time.time()))
    cv2.imwrite(image_path + filename, cv_image)

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print ("USAGE: python " + sys.argv[0] + " [path_to_folder]")
    else:
        global image_path
        image_path = sys.argv[1]
        rospy.init_node('image_subscriber')
        rospy.Subscriber('/carla/agent_0/rgb_view/image', Image, image_callback)
        rospy.spin()


