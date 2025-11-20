// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "lsa_nav_controller/core/road_scan_creator.hpp"
// #include "lsa_nav_controller/core/controller.hpp"
#include "lsa_nav_controller/core/open_place_checker.hpp"
#include "lsa_nav_controller/core/potential_controller.hpp"

namespace lsa_nav_controller
{
class LsaNavController : public rclcpp::Node{
    public:
    LsaNavController();
    ~LsaNavController();
    void cb_lsa_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
    void cb_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void declare_param(void);
    void init_param(void);
    void init_pubsub(void);
    void init_tf(void);
    void main_loop(void);
    int get_loop_freq(void);
    bool get_tf_pose(
        const std::string & target_frame, 
        const std::string & source_frame,
        geometry_msgs::msg::Pose2D & odom);
    void publish_cmd_vel(CmdVel & cmd_vel);

    private:
    // ROS2 Pub/Sub
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_lsa_map_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_road_scan_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_test_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    // Core Components
    std::unique_ptr<PotentialController> potential_controller_;
    std::unique_ptr<OpenPlaceChecker> open_place_checker_;
    //std::unique_ptr<RoadScanCreator> road_scan_creator_;
    // std::unique_ptr<Controller> controller_;

    // Sensor Components
    std::shared_ptr<Scan> scan_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    int control_freq_;
    std::string base_frame_id_, odom_frame_id_, scan_frame_id_;

    // variables
    bool init_tf_, receive_scan_, receive_map_;
};
    
} // namespace lsa_nav_controller
