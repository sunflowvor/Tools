import roslib
import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
 
rgb = '/media/ivan/Elements/yimin/0813/06(budiaozhen-30)/left_cam/'  #rgb path
bridge = CvBridge()
 
bag_dir = "/media/ivan/Elements/yimin/0813/06(budiaozhen-30)/bag"
files = os.listdir(bag_dir)
print(files)
for each in files:
    bag_file = os.path.join(bag_dir, each)
    print(bag_file)
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic,msg,t in bag.read_messages():
	    if topic == "/awe_ros/huidian/data":   #rgb topic
	        cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
	        timestr = "%.6f" %  msg.header.stamp.to_sec()   #rgb time stamp
	        image_name = timestr+ ".png"
	        cv2.imwrite(rgb + image_name, cv_image)
	        print("saving image:", rgb + image_name)
