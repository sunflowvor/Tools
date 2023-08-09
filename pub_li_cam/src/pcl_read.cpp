#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

struct EIGEN_ALIGN16 MyPoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


POINT_CLOUD_REGISTER_POINT_STRUCT(MyPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)


main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_read");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

    sensor_msgs::PointCloud2 output;
    
    
    pcl::PointCloud<MyPoint>::Ptr cloud (new pcl::PointCloud<MyPoint>);
    
    if (pcl::io::loadPCDFile<MyPoint> ("/home/ivan/catkin_ws_new/src/pub_li_cam/3Dbox-2/1665798188.999590400.pcd", *cloud) == -1)
    {
    	PCL_ERROR("Could not read file");
    	return (-1);
    }

    
    //pcl::io::loadPCDFile ("/home/ivan/catkin_ws_new/src/pub_li_cam/3Dbox-2/1665798188.999590400.pcd", cloud);

    //pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
