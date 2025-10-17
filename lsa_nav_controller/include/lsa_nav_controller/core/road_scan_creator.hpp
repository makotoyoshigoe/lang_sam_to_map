// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>

#include "lsa_nav_controller/map/map.hpp"

namespace lsa_nav_controller
{
class RoadScanCreator : public Map{
    public:
    RoadScanCreator(float max_angle_abs, float min_angle_abs, float angle_increment, float max_range, float min_range);
    ~RoadScanCreator();
    bool create_road_scan(geometry_msgs::msg::Pose2D & odom);
    void scanning_road_side(float start_deg, float end_deg);
    size_t rad_to_index(float rad);
    float index_to_rad(size_t index);
    float distance_from_occupied_grid(Grid gs, Grid ge);
    void get_scan_msg(sensor_msgs::msg::LaserScan & scan_msg);

    private:
    float max_angle_abs_, min_angle_abs_, angle_increment_;
    float max_range_, min_range_;
    float max_angle_, min_angle_;
    std::vector<float> ranges_;
    geometry_msgs::msg::Pose2D odom_;
};
    
} // namespace lsa_nav_controller
