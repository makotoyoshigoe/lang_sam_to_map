// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/core/road_scan_creator.hpp"
#include <cmath>

namespace lsa_nav_controller
{

RoadScanCreator::RoadScanCreator(
    float max_angle_abs, float min_angle_abs, float angle_increment, float max_range, float min_range)
: max_angle_abs_(max_angle_abs), min_angle_abs_(min_angle_abs), angle_increment_(angle_increment), 
  max_range_(max_range), min_range_(min_range), 
  Map()
{
    ranges_.assign(2*max_angle_abs/angle_increment, INFINITY);
    max_angle_ = max_angle_abs;
    min_angle_ = -max_angle_abs;
}

bool RoadScanCreator::create_road_scan(geometry_msgs::msg::Pose2D & odom)
{
    if(!init_map_receive_) return false;
    odom_ = odom;
    scanning_road_side(-max_angle_abs_, -min_angle_abs_);
    scanning_road_side(min_angle_abs_, max_angle_abs_);
    return true;
}

void RoadScanCreator::scanning_road_side(float start_deg, float end_deg)
{
    size_t is = rad_to_index(start_deg), ie = rad_to_index(end_deg);
    Grid gs = point_to_grid(odom_.x, odom_.y);
    for(size_t i=is; i<=ie; ++i){
        float theta = odom_.theta + index_to_rad(i);
        float xe = odom_.x + max_range_ * cos(theta);
        float ye = odom_.y + max_range_ * sin(theta);
        Grid ge = point_to_grid(xe, ye);
        ranges_[i] = distance_from_occupied_grid(gs, ge);
    }
<<<<<<< HEAD
    // RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Complete Road Scanning");
=======
>>>>>>> feat/publish-road-scan
}

size_t RoadScanCreator::rad_to_index(float rad){return (rad - min_angle_) / angle_increment_;}

float RoadScanCreator::index_to_rad(size_t index){return index * angle_increment_ + min_angle_;}

float RoadScanCreator::distance_from_occupied_grid(Grid gs, Grid ge)
{
    Grid g_cur = gs;
    Grid d{std::abs(ge.x - gs.x), std::abs(ge.y - gs.y)};
    Grid s{(gs.x < ge.x) ? 1 : -1, (gs.y < ge.y) ? 1 : -1};
    
    int err = d.x - d.y;

    while (true) {
        if (g_cur.x == ge.x && g_cur.y == ge.y) return max_range_;
        if (is_out_range(g_cur)){
            return max_range_;
        }
        if (data_[g_cur.x][g_cur.y] == 100){
            Grid gd = g_cur - gs;
            return hypot(gd.x, gd.y) * resolution_;
        }
        int e2 = 2 * err;
        if (e2 > -d.y) {err -= d.y; g_cur.x += s.x;}
        if (e2 <  d.x) {err += d.x; g_cur.y += s.y;}
    }
}
void RoadScanCreator::get_scan_msg(sensor_msgs::msg::LaserScan & scan_msg)
{
    scan_msg.angle_min = min_angle_;
    scan_msg.angle_max = max_angle_;
    scan_msg.angle_increment = angle_increment_;
    scan_msg.range_min = min_range_;
    scan_msg.range_max = max_range_ * 2;
    scan_msg.ranges = ranges_;
}

RoadScanCreator::~RoadScanCreator(){}
    
} // namespace lsa_nav_controller
