import rosbag 
import os

save_path = "/media/ivan/Elements/yimin/0812/gps/odom"
bag_file = "/media/ivan/Elements/yimin/0812/2022-08-12-15-04-44.bag"
bag = rosbag.Bag(bag_file, "r")
bag_data = bag.read_messages("/gps/odom")

for topic, msg, t in bag_data:
    secs = msg.header.stamp.secs
    nsecs = msg.header.stamp.nsecs
    p_x = msg.pose.pose.position.x
    p_y = msg.pose.pose.position.y
    p_z = msg.pose.pose.position.z

    o_x = msg.pose.pose.orientation.x
    o_y = msg.pose.pose.orientation.y
    o_z = msg.pose.pose.orientation.z
    o_w = msg.pose.pose.orientation.w

    t_lx = msg.twist.twist.linear.x
    t_ly = msg.twist.twist.linear.y
    t_lz = msg.twist.twist.linear.z

    t_ax = msg.twist.twist.angular.x
    t_ay = msg.twist.twist.angular.y
    t_az = msg.twist.twist.angular.z

    file_name = str(secs) + "." + str(nsecs) + ".txt"
    print("writting:", file_name)
    file_name = os.path.join(save_path, file_name)

    context = "pose:" + "\n" + "  position:" + "\n"  + "    x:" + str(p_x) + "\n" \
              + "    y:" + str(p_y) + "\n" + "    z:" + str(p_z) + "\n" \
              + "  orientation:"  + "\n"  + "    x:" + str(o_x) + "\n" \
              + "    y:" + str(o_y) + "\n" + "    z:" + str(o_z) + "\n" \
              + "    w:" + str(o_w) + "\n" \
              + "twist:" + "\n" + "  linear:" + "\n"  + "    x:" + str(t_lx) + "\n" \
              + "    y:" + str(t_ly) + "\n" + "    z:" + str(t_lz) + "\n" \
              + "  angular:" + "\n"  + "    x:" + str(t_ax) + "\n" \
              + "    y:" + str(t_ay) + "\n" + "    z:" + str(t_az)
             
    with open (file_name, "w") as gps_file:
        gps_file.write(context)
    gps_file.close()
