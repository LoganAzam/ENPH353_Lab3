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

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gauss = cv2.GaussianBlur(gray, (5, 5), 0)
        
        _, thresh = cv2.threshold(gauss, 75, 255, cv2.THRESH_BINARY_INV)

        moments = cv2.moments(thresh)
        move = Twist()

        if moments['m00'] > 0:
            cx = int(moments['m10']/moments['m00'])
            width = cv_image.shape[1]
            error = cx - width/2
            
            move.linear.x = 0.2
            move.angular.z = -float(error) / 90
        else:
            move.linear.x = 0
            move.angular.z = 0.3

        self.cmd_pub.publish(move)

        # Debugging
        # cv2.imshow("Centroid View", thresh)
        # cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('line_follower_node')
    try:
        # Match the class name precisely
        follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
