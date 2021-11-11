#include<iostream>
#include<pcl/io/pcd_io.h>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>


int main(int argc, char* argv[]){
  std::string map_path;
  ros::init(argc, argv, "map_publisher");
  ros::NodeHandle nh("~");
  nh.param<std::string>("map_path", map_path, "/home/biomotion/itri_map.pcd");

  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
  sensor_msgs::PointCloud2::Ptr map_cloud(new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_points(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *map_points);
  pcl::toROSMsg(*map_points, *map_cloud);
  map_cloud->header.frame_id = "world";

  ros::Duration(0.1).sleep();

  while(ros::ok()){
    ROS_INFO("pub map");
    pub_map.publish(*map_cloud);
    ros::Duration(5.).sleep();
  }
}