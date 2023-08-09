import rosbag 
import os

save_path = "/media/ivan/Elements/yimin/0812/imu"
bag_file = "/media/ivan/Elements/yimin/0812/2022-08-12-15-04-44.bag"
bag = rosbag.Bag(bag_file, "r")
bag_data = bag.read_messages("/an_device/Imu")

for topic, msg, t in bag_data:
    secs = msg.header.stamp.secs
    nsecs = msg.header.stamp.nsecs

    o_x = msg.orientation.x
    o_y = msg.orientation.y
    o_z = msg.orientation.z
    o_w = msg.orientation.w

    a_x = msg.angular_velocity.x
    a_y = msg.angular_velocity.y
    a_z = msg.angular_velocity.z

    l_x = msg.linear_acceleration.x
    l_y = msg.linear_acceleration.y
    l_z = msg.linear_acceleration.z

    file_name = str(secs) + "." + str(nsecs) + ".txt"
    print("writting:", file_name)
    file_name = os.path.join(save_path, file_name)

    context = "orientation:" + "\n" + "  x:" + str(o_x) + "\n" \
              + "  y:" + str(o_y) + "\n" + "  z:" + str(o_z) + "\n" \
              + "  w:" + str(o_w) + "\n" \
              + "angular_velocity:" + "\n" + "  x:" + str(a_x) + "\n" \
              + "  y:" + str(a_y) + "\n" + "  z:" + str(a_z) + "\n" \
              + "linear_acceleration:" + "\n"  + "  x:" + str(l_x) + "\n" \
              + "  y:" + str(l_y) + "\n" + "  z:" + str(l_z)
             
    with open (file_name, "w") as gps_file:
        gps_file.write(context)
    gps_file.close()
