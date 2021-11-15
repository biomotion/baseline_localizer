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

## How to Use

- [prepare your data](#prepare-data)
- [start launch file](#launch-file)
- [play rosbag with slow speed](#play-rosbag)
- [check your result](#check-result)

### Prepare Data
First, you should download your pcd map to your computer and modify map_path in `pub_map_node.cpp` for bag from ITRI.

For nuscenes datasets, you should utilize [map_tile_loader](https://github.com/biomotion/map_tile_loader) package and setup your nuscenes map files.

### Launch File
Start launch file by,  
```bash
roslaunch baseline_localizer itri.launch
# or
roslaunch baseline_localizer nuscenes.launch
```

### Play Rosbag

Play rosbag with the following command.
```bash
rosbag play --pause -r 0.1 <your_bag_file>
```
You can pause/resume the playing process by pressing white space on the rosbag play terminal.

It is recommanded to pause a while when the localizer node is initializing. 

### Check Result

The results will be saved in `results/` folder by default. Check if there are correct number of lines and upload your result to our competition servers.
