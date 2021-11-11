# Baseline Localizer

### Auther: Biomotion 
### Create Date: 2021/10/28
### Last Modified: 2021/11/01

## Dependencies
- geometry_msgs
- pcl_ros
- sensor_msgs
- tf2
- tf2_ros
- tf_conversions

## Nodes
- pub_map
  - parameters: map_path
  - subscribe: N/A
  - publish: /map (sensor_msgs::PointCloud2)
  
- localizer
  - parameters: baselink2lidar_trans (float array), baselink2lidar_rot (float array), result_save_path (string)
  - subscribe: /map (sensor_msgs::PointCloud2), /lidar_points (sensor_msgs::PointCloud2), /gps (geometry_msgs::PointStamped)
  - publish: /lidar_pose (geometry_msgs::PoseStamped), /transformed_points (sensor_msgs::PointCloud2)
  - output: result poses as csv file saved in `result_save_path`