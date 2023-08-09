#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>  

int i = 0;

using namespace std;
  
void cloudCB(const sensor_msgs::PointCloud2 &input)  
{  
  pcl::PointCloud<pcl::PointXYZ> cloud;  
  pcl::fromROSMsg(input, cloud);//从ROS类型消息转为PCL类型消息  
  std::string save_path = "/media/ivan/waytous-2T/scene_02/lidar_test/00/" + std::to_string(i) + ".pcd";
  pcl::io::savePCDFileASCII (save_path, cloud);//保存pcd  
  i += 1;
}  
int main(int argc, char ** argv)
{  
  ros::init (argc, argv, "pcl_write");  
  ros::NodeHandle nh;  
  ros::Subscriber bat_sub = nh.subscribe("/os_cloud_node/points", 10000, cloudCB);//接收点云  
  ros::spin();  
  return 0;  
}  


