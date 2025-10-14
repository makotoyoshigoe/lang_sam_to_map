// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

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

    private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_lsa_map_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_road_scan_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_test_;
};
    
} // namespace lsa_nav_controller
