#!/usr/bin/env python
#coding:utf-8

import rospy
import sys
sys.path.append('.')
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def pubImage():
    rospy.init_node('pubImage',anonymous = True)
    pub = rospy.Publisher('ShowImage', Image, queue_size = 10)
    rate = rospy.Rate(10)
    bridge = CvBridge()
    gt_imdb = []
    imagepath = "/media/ivan/My Passport/3Dbox-2/1665798189.000000.png"

    image = cv2.imread(imagepath)
    #image = cv2.resize(image,(900,450))
    pub.publish(bridge.cv2_to_imgmsg(image,"bgr8"))
    #cv2.imshow("lala",image)
    #cv2.waitKey(0)
    rate.sleep()

if __name__ == '__main__':
    try:
        pubImage()
    except rospy.ROSInterruptException:
        pass
