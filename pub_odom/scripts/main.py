#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import utm

beijing_latitude = 40
beijing_longtitude = 116
beijing_altitude = 43

halaer_latitude = 49.3902600
halaer_longtitude = 119.7717000
halaer_altitude = 610

beijing_utm = utm.from_latlon(beijing_latitude, beijing_longtitude)
halaer_utm = utm.from_latlon(halaer_latitude, halaer_longtitude)

pub = rospy.Publisher('/new_odom', Odometry, queue_size=10)


def callback(data):
    header = data.header
    child_frame_id = data.child_frame_id
    pose = data.pose
    twist = data.twist
    posese = pose.pose
    covariance = pose.covariance
    position = posese.position
    orientation = posese.orientation

    x = position.x
    y = position.y
    z = position.z

    utm_e = x + beijing_utm[0]
    utm_n = y + beijing_utm[1]
    z = z - beijing_altitude
    new_x = utm_e - halaer_utm[0]
    new_y = utm_n - halaer_utm[1]

    new_odom_data = Odometry()
    new_odom_data.header = header
    new_odom_data.child_frame_id = child_frame_id
    new_odom_data.twist = twist
    posese.position.x = new_x
    posese.position.y = new_y
    posese.position.z = z
    new_odom_data.pose.pose = posese
    new_odom_data.pose.covariance = covariance

    
    pub.publish(new_odom_data)







def subscribe_pub():
    rospy.Subscriber("/gps/odom", Odometry, callback)
    #rospy.init_node('subscribe_and_pub', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('subscribe_and_pub', anonymous=True)
    subscribe_pub()
