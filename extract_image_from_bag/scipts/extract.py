import roslib
import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
 
rgb = '/home/ivan/Documents/gps/3/left_cam/'  #rgb path
bridge = CvBridge()
 
with rosbag.Bag('/home/ivan/Documents/gps/3/2022-08-13-17-01-38.bag', 'r') as bag:
    for topic,msg,t in bag.read_messages():
        if topic == "/awe_ros/huidian/data":   #rgb topic
            cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
            timestr = "%.6f" %  msg.header.stamp.to_sec()   #rgb time stamp
            image_name = timestr+ ".png"
            cv2.imwrite(rgb + image_name, cv_image)
            print("saving image:", rgb + image_name)
