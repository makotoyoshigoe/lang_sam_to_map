// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/sensor/road_scan.hpp"

namespace lsa_nav_controller
{

RoadScan::RoadScan(
    float max_angle_abs, float min_angle_abs, float angle_increment, 
    float max_range, float min_range)
: max_angle_abs_(max_angle_abs), min_angle_abs_(min_angle_abs), angle_increment_(angle_increment), 
  max_range_(max_range), min_range_(min_range)
{

}

void RoadScan::get_road_scan(
    std::vector<float> ranges, sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    scan_msg->angle_max = max_angle_abs_;
    scan_msg->angle_min = -max_angle_abs_;
    scan_msg->angle_increment = angle_increment_;
    scan_msg->range_max = max_range_;
    scan_msg->range_min = min_range_;
    scan_msg->ranges = ranges;
}

RoadScan::~RoadScan(){}
    
} // namespace lsa_nav_controller
