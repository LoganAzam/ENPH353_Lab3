#! /usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class LineFollower:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/robot/camera1/image_raw", Image, self.callback, queue_size=3)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # --- Everything below must be indented under callback ---
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gauss = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Note: cv2.threshold returns two values; we use the second one
        _, thresh = cv2.threshold(gauss, 75, 255, cv2.THRESH_BINARY_INV)

        M = cv2.moments(thresh)
        move = Twist()

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            width = cv_image.shape[1]
            error = cx - width/2
            
            # P-Controller logic
            move.linear.x = 0.2
            move.angular.z = -float(error) / 100
        else:
            # Search for the line
            move.linear.x = 0
            move.angular.z = 0.5

        self.cmd_pub.publish(move)

        # Optional Debug View
        cv2.imshow("Centroid View", thresh)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('line_follower_node')
    try:
        # Match the class name precisely
        follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
