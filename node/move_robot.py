#! /usr/bin/env python3

## @package move_robot
#  This module is for Lab 3 ENPH353 on line following and ROS
#
#  This node processes camera data from Gazebo to identify a line 
#  on the Monza track and publishes velocity commands to follow it.

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

## LineFollower is a class to take in video, process it and return steering instructions
#
#  Handles the initialization of ROS interfaces and the OpenCV logic
#  required to calculate robot steering.
class LineFollower:

    ## The constructor.
    #  Sets up the CV Bridge and defines the Subscriber and Publisher.
    def __init__(self):
        ## @var bridge
        #  An instance of CvBridge to convert ROS images to OpenCV.
        self.bridge = CvBridge()
        
        ## @var image_sub
        #  The ROS subscriber for the raw camera feed.
        self.image_sub = rospy.Subscriber("/robot/camera1/image_raw", Image, self.callback, queue_size=3)
        
        ## @var cmd_pub
        #  The ROS publisher for sending velocity commands to the robot.
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    ## Documentation for the callback method.
    #  @param self The object pointer.
    #  @param data The ROS image message received from the camera.
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Processing the image to find the line
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gauss = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Binary thresholding to isolate the black path
        _, thresh = cv2.threshold(gauss, 75, 255, cv2.THRESH_BINARY_INV)

        # Moments determines the centroid of the track pixels
        moments = cv2.moments(thresh)
        move = Twist()

        if moments['m00'] > 0:
            # Calculating the horizontal centroid
            cx = int(moments['m10']/moments['m00'])
            width = cv_image.shape[1]
            error = cx - width/2
            
            # P-Controller logic
            move.linear.x = 0.2
            move.angular.z = -float(error) / 90
        else:
            # Default behavior if the line is not in view
            move.linear.x = 0
            move.angular.z = 0.3

        self.cmd_pub.publish(move)

        # Debugging
        # cv2.imshow("Centroid View", thresh)
        # cv2.waitKey(1)

if __name__ == '__main__':
    ## Main execution block.
    #  Initializes the node and instantiates the follower.
    rospy.init_node('line_follower_node')
    try:
        follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
