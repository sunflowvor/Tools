#!/usr/bin/env python
#coding:utf-8

import rospy
import sys
sys.path.append('.')
import cv2
import os
import numpy as np
import sensor_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pcl

from sensor_msgs.msg import PointCloud2
import open3d as o3d
from sensor_msgs.msg import PointField

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)

def pubImage():
    #rospy.init_node('pubImage',anonymous = True)
    pub = rospy.Publisher('/awe_ros/huidian/data', Image, queue_size = 10)
    rate = rospy.Rate(10)
    bridge = CvBridge()
    gt_imdb = []
    imagepath = "/home/ivan/catkin_ws_new/src/pub_li_cam/3Dbox-2/1666420699.000000.png"

    image = cv2.imread(imagepath)

    pub_c = rospy.Publisher('/os_cloud_node/points', PointCloud2, queue_size = 10)
    pcd_load = o3d.io.read_point_cloud("/home/ivan/catkin_ws_new/src/pub_li_cam/3Dbox-2/1666420698.999311872.pcd")
    #o3d.visualization.draw_geometries([pcd_load])
    
    xyz_load = np.asarray(pcd_load.points)

    #xyz_load=np.array([[225.0, -71.0, 819.8],[237.0, -24.0, 816.0],[254.0, -82.0, 772.3]])
    #pointcloud_trans = pcl.PointCloud(xyz_load)

    # get data
    msg = PointCloud2()
    msg = convertCloudFromOpen3dToRos(pcd_load)
    #msg.header.stamp = rospy.Time().now()
    #msg.header.frame_id = "odom"
    #print(xyz_load)
    #print(xyz_load.shape)


    #msg.height = xyz_load.shape[1]
    #msg.width = xyz_load.shape[0]



    #msg.is_bigendian = False
    #msg.is_dense = False
    #msg.data = np.asarray(xyz_load, np.float32).tostring()
    

    #image = cv2.resize(image,(900,450))
    while not rospy.is_shutdown():
        pub.publish(bridge.cv2_to_imgmsg(image,"bgr8"))
        print("Image published...")
        pub_c.publish(msg)
        print("Cloud published...")
        #cv2.imshow("lala",image)
        #cv2.waitKey(0)
        #rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pub',anonymous = True)
    rate = rospy.Rate(10)
    pubImage()
