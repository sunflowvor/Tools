import rosbag 
import os

save_path = "/media/ivan/Elements/yimin/0812/gps/fix"
bag_file = "/media/ivan/Elements/yimin/0812/2022-08-12-15-04-44.bag"
bag = rosbag.Bag(bag_file, "r")
bag_data = bag.read_messages("/gps/fix")


for topic, msg, t in bag_data:
    secs = msg.header.stamp.secs
    nsecs = msg.header.stamp.nsecs
    latitude = msg.latitude
    longitude = msg.longitude
    altitude = msg.altitude

    file_name = str(secs) + "." + str(nsecs) + ".txt"
    print("writting:", file_name)
    file_name = os.path.join(save_path, file_name)

    context = "latitude:" + " " + str(latitude) + "\n" \
             + "longitude:" + " " + str(longitude) + "\n" \
             + "altitude:" + " " + str(altitude)
    with open (file_name, "w") as gps_file:
        gps_file.write(context)
    gps_file.close()
