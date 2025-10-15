// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lsa_nav_controller/core/road_scan_creator.hpp"

namespace lsa_nav_controller
{
class LsaNavController : public rclcpp::Node{
    public:
    LsaNavController();
    ~LsaNavController();
    void cb_lsa_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
    void declare_param(void);
    void init_param(void);
    void init_pubsub(void);
    void init_tf(void);
    void main_loop(void);
    int get_loop_freq(void);
    void publish_road_scan(void);
    bool get_odom(geometry_msgs::msg::Pose2D & odom);
    private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_lsa_map_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_road_scan_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_test_;
    std::unique_ptr<RoadScanCreator> road_scan_creator_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    int control_freq_;
    std::string base_frame_id_, odom_frame_id_;
    bool init_tf_;
};
    
} // namespace lsa_nav_controller
