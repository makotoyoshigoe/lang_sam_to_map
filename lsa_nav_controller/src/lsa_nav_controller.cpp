// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/lsa_nav_controller.hpp"
#include "lsa_nav_controller/map/map.hpp"

namespace lsa_nav_controller
{
LsaNavController::LsaNavController() : Node("lsa_nav_controller")
{
    declare_param();
    init_param();
    init_pubsub();
}

LsaNavController::~LsaNavController(){}

void LsaNavController::declare_param(void)
{
    declare_parameter("road_scan.max_angle_abs", 30.0);
    declare_parameter("road_scan.min_angle_abs", 60.0);
    declare_parameter("road_scan.angle_increment", 0.25);
    declare_parameter("road_scan.max_range", 5.);
    declare_parameter("road_scan.min_range", 0.1);
}

void LsaNavController::init_param(void)
{
    float max_angle = get_parameter("road_scan.max_angle_abs").as_double();
    float min_angle = get_parameter("road_scan.min_angle_abs").as_double();
    float angle_increment = get_parameter("road_scan.angle_increment").as_double();
    float max_range = get_parameter("road_scan.max_range").as_double();
    float min_range = get_parameter("road_scan.min_range").as_double();
}

void LsaNavController::init_pubsub(void)
{
    sub_lsa_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "lang_sam_map", rclcpp::QoS(10), std::bind(&LsaNavController::cb_lsa_map, this, std::placeholders::_1));
    pub_road_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("scan/road", rclcpp::QoS(10));
    pub_map_test_ = create_publisher<nav_msgs::msg::OccupancyGrid>("lsa_map/test", rclcpp::QoS(10));
}

void LsaNavController::cb_lsa_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    std::unique_ptr<Map> map;
    map = std::make_unique<Map>();
    map->update_map(msg);
    // map->fill_bottom();
    // nav_msgs::msg::OccupancyGrid map_msg;
    // map->get_map_msg(map_msg);
    // pub_map_test_->publish(map_msg);
}
    
} // namespace lsa_nav_controller

