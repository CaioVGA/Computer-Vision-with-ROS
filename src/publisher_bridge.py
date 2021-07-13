#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
import cv2
import numpy as np
import os
import imutils
from cv_bridge import CvBridge, CvBridgeError

IMAGE_WIDTH=1241
IMAGE_HEIGHT=376

image_pubulish = rospy.Publisher('receba', Image, queue_size = 10)

rospy.init_node('image_converter', anonymous = True)

rate = rospy.Rate(1)

img = cv2.imread('../images/drone2.jpg')
# imag = cv2.resize(img, (500,500))
cv2.namedWindow("Image",cv2.WINDOW_NORMAL)
cv2.imshow("Image", img)


def talker():
    while not rospy.is_shutdown():
        publish_image(img)
        rospy.loginfo('imagem enviada')
        rate.sleep()

def publish_image(imgdata):
    # image_temp=Image()
    # header = Header(stamp=rospy.Time.now())
    # header.frame_id = 'map'
    # image_temp.height=IMAGE_HEIGHT
    # image_temp.width=IMAGE_WIDTH
    # image_temp.encoding='rgb8'
    # image_temp.data=np.array(imgdata).tostring()
    # #print(imgdata)
    # #image_temp.is_bigendian=True
    # image_temp.header=header
    # image_temp.step=1241*3
    cv_image = CvBridge().cv2_to_imgmsg(imgdata,'bgr8')
    image_pubulish.publish(cv_image)
    #print(cv_image)


    # img_np_arr = np.fromstring(imgdata, np.uint8)
    # image_np = cv2.imdecode(img_np_arr, cv2.CV_LOAD_IMAGE_COLOR)

    # msg = CompressedImage()
    # msg.header.stamp = rospy.Time.now()
    # msg.format = "jpeg"
    # msg.data = np.array(cv2.imencode('.jpg', imgdata)[1]).tostring()
    # # Publish new image
    # image_pubulish.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass