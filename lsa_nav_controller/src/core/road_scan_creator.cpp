// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/core/road_scan_creator.hpp"
#include <cmath>

namespace lsa_nav_controller
{

RoadScanCreator::RoadScanCreator(
    float max_angle_abs, float min_angle_abs, float angle_increment, float max_range, float min_range, float front_angle_abs)
: Map(), max_angle_abs_(max_angle_abs), min_angle_abs_(min_angle_abs), angle_increment_(angle_increment), 
  max_range_(max_range), min_range_(min_range), max_angle_(max_angle_abs), min_angle_(-max_angle_abs), front_angle_abs_(front_angle_abs) 
{
    ranges_.assign(2*max_angle_abs/angle_increment, INFINITY);
}

bool RoadScanCreator::create_road_scan(void)
{
    if(!init_map_receive_) return false;
    scanning_road_side(-max_angle_abs_, -min_angle_abs_);
    scanning_road_side(min_angle_abs_, max_angle_abs_);
    scanning_road_side(-front_angle_abs_, front_angle_abs_);
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
}

size_t RoadScanCreator::rad_to_index(float rad){return (rad - min_angle_) / angle_increment_;}

float RoadScanCreator::index_to_rad(size_t index){return index * angle_increment_ + min_angle_;}

float RoadScanCreator::distance_from_occupied_grid(Grid gs, Grid ge)
{
    Grid g_cur = gs;
    Grid d{std::abs(ge.x - gs.x), std::abs(ge.y - gs.y)};
    Grid s{(gs.x < ge.x) ? 1 : -1, (gs.y < ge.y) ? 1 : -1};
    
    int err = d.x - d.y;
    float result = max_range_;

    while (true) {
        if (g_cur.x == ge.x && g_cur.y == ge.y) return result;
        if (is_out_range(g_cur)) return result;
        if (data_[g_cur.x][g_cur.y] == 0) result = get_dist_two_grids(gs, g_cur);
        if (data_[g_cur.x][g_cur.y] == 100) return get_dist_two_grids(gs, g_cur);
        int e2 = 2 * err;
        if (e2 > -d.y) {err -= d.y; g_cur.x += s.x;}
        if (e2 <  d.x) {err += d.x; g_cur.y += s.y;}
    }
}
float RoadScanCreator::get_dist_two_grids(Grid g1, Grid g2)
{
    Grid gd = g2 - g1;
    return hypot(gd.x, gd.y) * resolution_;
}

void RoadScanCreator::get_average_ranges(
    geometry_msgs::msg::Pose2D & odom, 
    float & right, float & left, float & front)
{
    odom_ = odom;
    create_road_scan();
    right = get_abs_ave_lateral(-max_angle_abs_, -min_angle_abs_);
    left = get_abs_ave_lateral(min_angle_abs_, max_angle_abs_);
    front = get_abs_ave_vertical(-front_angle_abs_, front_angle_abs_);
}

float RoadScanCreator::get_abs_ave_lateral(float start_rad, float end_rad)
{
    size_t is = rad_to_index(start_rad), ie = rad_to_index(end_rad);
    float sum = 0, n = 0, rad = start_rad;
    for(size_t i = is; i <= ie; ++i){
        sum += ranges_[i] * fabs(sin(odom_.theta + rad));
        ++n;
        rad += angle_increment_;
    }
    return sum / n;
}

float RoadScanCreator::get_abs_ave_vertical(float start_rad, float end_rad)
{
    size_t is = rad_to_index(start_rad), ie = rad_to_index(end_rad);
    float sum = 0, n = 0, rad = start_rad;
    for(size_t i = is; i <= ie; ++i){
        sum += ranges_[i] * fabs(cos(odom_.theta + rad));
        ++n;
        rad += angle_increment_;
    }
    return sum / n;
}

RoadScanCreator::~RoadScanCreator(){}
    
} // namespace lsa_nav_controller
