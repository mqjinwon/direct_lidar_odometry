/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/times.h>

#include <atomic>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/camera_info.h>
#include <sensor_msgs/msg/image.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/algorithm/string.hpp>
#include <boost/circular_buffer.hpp>
#include <direct_lidar_odometry/srv/save_pcd.hpp>
#include <direct_lidar_odometry/srv/save_traj.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nano_gicp/nano_gicp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dlo/utils.h"
#include "rclcpp/rclcpp.hpp"

typedef pcl::PointXYZI PointType;

namespace dlo {

class OdomNode;
class MapNode;

}  // namespace dlo
