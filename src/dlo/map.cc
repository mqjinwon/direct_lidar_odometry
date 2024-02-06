/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/map.h"

std::atomic<bool> dlo::MapNode::abort_(false);

/**
 * Constructor
 **/

dlo::MapNode::MapNode() : Node("dlo_map_node") {
    this->getParams();

    this->abort_timer =
        this->create_wall_timer(std::chrono::duration<double>(0.01),
                                std::bind(&dlio::MapNode::abortTimerCB, this));

    if (this->publish_full_map_) {
        this->publish_timer = this->create_wall_timer(
            std::chrono::duration<double>(this->publish_freq_),
            std::bind(&dlio::MapNode::publishTimerCB, this));
    }

    this->keyframe_sub = this->create_subscription<sensor_msgs::PointCloud2>(
        "keyframes", 1,
        std::bind(&dlo::MapNode::keyframeCB, this, std::placeholders::_1));
    this->map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("map", 1);

    this->save_pcd_srv =
        this->nh.advertiseService("save_pcd", &dlo::MapNode::savePcd, this);

    // initialize map
    this->dlo_map =
        pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);

    RCLCPP_INFO(this->get_logger(), "DLO Map Node Initialized");
}

/**
 * Destructor
 **/

dlo::MapNode::~MapNode() {}

/**
 * Get Params
 **/

void dlo::MapNode::getParams() {
    this->declare_parameter<std::string>("odom/odom_frame", "odom");
    this->declare_parameter<double>("map/sparse/frequency", 1.0);
    this->declare_parameter<double>("map/sparse/leafSize", 0.5);

    this->get_parameter("odom/odom_frame", this->odom_frame);
    this->get_parameter("map/sparse/frequency", this->publish_freq_);
    this->get_parameter("map/sparse/leafSize", this->leaf_size_);

    // ros2 namespace
    std::string ns = this->get_namespace();
    ns.erase(0, 1);
    this->odom_frame = ns + "/" + this->odom_frame;
}

/**
 * Start Map Node
 **/

void dlo::MapNode::start() { ROS_INFO("Starting DLO Map Node"); }

/**
 * Stop Map Node
 **/

void dlo::MapNode::stop() {
    RCLCPP_WARN(this->get_logger(), "Stopping DLO Map Node");

    // shutdown
    rclcpp::shutdown();
}

/**
 * Abort Timer Callback
 **/

void dlo::MapNode::abortTimerCB() {
    if (abort_) {
        stop();
    }
}

/**
 * Publish Timer Callback
 **/

void dlo::MapNode::publishTimerCB() {
    if (this->dlo_map->points.size() ==
        this->dlo_map->width * this->dlo_map->height) {
        sensor_msgs::PointCloud2 map_ros;
        pcl::toROSMsg(*this->dlo_map, map_ros);
        map_ros.header.stamp = this->now();
        map_ros.header.frame_id = this->odom_frame;
        this->map_pub->publish(map_ros);
    }
}

/**
 * Node Callback
 **/

void dlo::MapNode::keyframeCB(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe) {
    // convert scan to pcl format
    pcl::PointCloud<PointType>::Ptr keyframe_pcl =
        pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*keyframe, *keyframe_pcl);

    // voxel filter
    this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_,
                                this->leaf_size_);
    this->voxelgrid.setInputCloud(keyframe_pcl);
    this->voxelgrid.filter(*keyframe_pcl);

    // save keyframe to map
    this->map_stamp = keyframe->header.stamp;
    *this->dlo_map += *keyframe_pcl;

    if (!this->publish_full_map_) {
        if (keyframe_pcl->points.size() ==
            keyframe_pcl->width * keyframe_pcl->height) {
            sensor_msgs::PointCloud2 map_ros;
            pcl::toROSMsg(*keyframe_pcl, map_ros);
            map_ros.header.stamp = this->now();
            map_ros.header.frame_id = this->odom_frame;
            this->map_pub->publish(map_ros);
        }
    }
}

bool dlo::MapNode::savePcd(
    direct_lidar_odometry::srv::save_pcd::Request& req,
    direct_lidar_odometry::srv::save_pcd::Response& res) {
    pcl::PointCloud<PointType>::Ptr m = pcl::PointCloud<PointType>::Ptr(
        boost::make_shared<pcl::PointCloud<PointType>>(*this->dlo_map));

    float leaf_size = req.leaf_size;
    std::string p = req.save_path;

    std::cout << std::setprecision(2) << "Saving map to " << p + "/dlo_map.pcd"
              << "... ";
    std::cout.flush();

    // voxelize map
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.setInputCloud(m);
    vg.filter(*m);

    // save map
    int ret = pcl::io::savePCDFileBinary(p + "/dlo_map.pcd", *m);
    res.success = ret == 0;

    if (res.success) {
        std::cout << "done" << std::endl;
    } else {
        std::cout << "failed" << std::endl;
    }

    return res.success;
}
