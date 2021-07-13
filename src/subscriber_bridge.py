#!/usr/bin/env python

import rospy 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def image_callback(ros_image):
    print('got an image')
    global bridge
    #convet ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image,'bgr8')
    except CvBridgeError as e:
        print(e)
    #from now on, you can work exactly like with opencv
    (rows,cols,channels) = cv_image.shape
    if cols > 200 and rows > 200:
        cv2.circle(cv_image, (100,100),90,255)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(cv_image,'Webcam Activated with ROS & OpenCV!',(10,350),font,1,(255,255,255),2,cv2.LINE_AA)
    cv2.imshow('Image Window', cv_image)


rospy.init_node('imagem_converter', anonymous = True)

image_sub = rospy.Subscriber('receba', Image, image_callback)

rate = rospy.Rate(1)

rate.sleep()
    
while not rospy.is_shutdown():
    rospy.spin()

cv2.destroyAllWindows()

