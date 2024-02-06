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
                                std::bind(&dlo::MapNode::abortTimerCB, this));

    if (this->publish_full_map_) {
        this->publish_timer = this->create_wall_timer(
            std::chrono::duration<double>(this->publish_freq_),
            std::bind(&dlo::MapNode::publishTimerCB, this));
    }

    this->keyframe_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto keyframe_sub_opt = rclcpp::SubscriptionOptions();
    keyframe_sub_opt.callback_group = this->keyframe_cb_group;

    this->keyframe_sub =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "keyframe", 1,
            std::bind(&dlo::MapNode::keyframeCB, this, std::placeholders::_1),
            keyframe_sub_opt);

    this->map_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);

    this->save_pcd_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    this->save_pcd_srv =
        this->create_service<direct_lidar_odometry::srv::SavePCD>(
            "save_pcd",
            std::bind(&dlo::MapNode::savePcd, this, std::placeholders::_1,
                      std::placeholders::_2),
            rmw_qos_profile_services_default, this->save_pcd_cb_group);

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
    dlo::declare_param(this, "dlo/mapNode/publishFullMap",
                       this->publish_full_map_, true);
    dlo::declare_param(this, "dlo/mapNode/odom_frame", this->odom_frame,
                       "odom");
    dlo::declare_param(this, "dlo/mapNode/publishFreq", this->publish_freq_,
                       1.0);
    dlo::declare_param(this, "dlo/mapNode/leafSize", this->leaf_size_, 0.25);

    // ros2 namespace
    std::string ns = this->get_namespace();
    ns.erase(0, 1);
    this->odom_frame = ns + "/" + this->odom_frame;
}

/**
 * Start Map Node
 **/

void dlo::MapNode::start() {
    RCLCPP_INFO(this->get_logger(), "Starting DLO Map Node");
}

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
        sensor_msgs::msg::PointCloud2 map_ros;
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
            sensor_msgs::msg::PointCloud2 map_ros;
            pcl::toROSMsg(*keyframe_pcl, map_ros);
            map_ros.header.stamp = this->now();
            map_ros.header.frame_id = this->odom_frame;
            this->map_pub->publish(map_ros);
        }
    }
}

bool dlo::MapNode::savePcd(
    std::shared_ptr<direct_lidar_odometry::srv::SavePCD::Request> req,
    std::shared_ptr<direct_lidar_odometry::srv::SavePCD::Response> res) {
    pcl::PointCloud<PointType>::Ptr m =
        std::make_shared<pcl::PointCloud<PointType>>(*this->dlo_map);

    float leaf_size = req->leaf_size;
    std::string p = req->save_path;

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
    res->success = ret == 0;

    if (res->success) {
        std::cout << "done" << std::endl;
    } else {
        std::cout << "failed" << std::endl;
    }

    return res->success;
}
