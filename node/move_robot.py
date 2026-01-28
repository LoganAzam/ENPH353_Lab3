#! /usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class image_converter:

	def __init__(self):
	  self.bridge = CvBridge()
	  self.image_sub = rospy.Subscriber(" ", Image, self.callback, queue_size=3)

	def callback(self, data):
	  try:
	    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	  except CvBridgeError as e:
		print(e)

class LineFollower:
    def __init__(self):


    while not rospy.is_shutdown():
        pub.publish(move)
        rate.sleep()

if __name__ == '__main__':
    try:
        
    except rospy.ROSInterruptException:
        pass
