/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/dlo.h"

class dlo::MapNode: public rclcpp::Node {
   public:
    MapNode();
    ~MapNode();

    static void abort() { abort_ = true; }

    void start();
    void stop();

   private:
    void abortTimerCB();
    void publishTimerCB();

    void keyframeCB(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe);

    bool savePcd(direct_lidar_odometry::save_pcd::Request & req,
                 direct_lidar_odometry::save_pcd::Response & res);

    void getParams();

    rclcpp::TimerBase::SharedPtr abort_timer;
    rclcpp::TimerBase::SharedPtr publish_timer;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

    rclcpp::Service<direct_lidar_inertial_odometry::srv::save_pcd>::SharedPtr save_pcd_srv;

    pcl::PointCloud<PointType>::Ptr dlo_map;
    pcl::VoxelGrid<PointType> voxelgrid;

    rclcpp::Time map_stamp;
    std::string odom_frame;

    bool publish_full_map_;
    double publish_freq_;
    double leaf_size_;

    static std::atomic<bool> abort_;
};
