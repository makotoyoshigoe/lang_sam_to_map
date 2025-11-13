// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/core/potential_controller.hpp"

namespace lsa_nav_controller
{
PotentialController::PotentialController(
    float critical_distance, 
    float replusive_gain_map, 
    float replusive_gain_scan, 
    float attractive_gain, 
    float detect_angle_start,
    float detect_angle_end,
    int detect_angle_division_num)
: critical_distance_(critical_distance), 
  replusive_gain_map_(replusive_gain_map), 
  replusive_gain_scan_(replusive_gain_scan),
  attractive_gain_(attractive_gain)
{
    scan_ = std::make_unique<Scan>(detect_angle_start, detect_angle_end, detect_angle_division_num);
    map_ = std::make_unique<Map>();
}

void PotentialController::set_scan_data(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    // Implementation to set scan data
    scan_->angle_min_ = scan_msg->angle_min;
    scan_->angle_max_ = scan_msg->angle_max;
    scan_->angle_increment_ = scan_msg->angle_increment;
    scan_->range_min_ = scan_msg->range_min;
    scan_->range_max_ = scan_msg->range_max;
    scan_->frame_id_ = scan_msg->header.frame_id;
    if(scan_->ranges_.size() != scan_msg->ranges.size()){
        scan_->ranges_.clear();
        scan_->ranges_.resize(scan_msg->ranges.size());
    }
    std::copy(
        scan_msg->ranges.begin(),
        scan_msg->ranges.end(),
        std::back_inserter(scan_->ranges_));
}

void PotentialController::set_map_data(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg)
{
    map_->frame_id_ = map_msg->header.frame_id;
    map_->height_ = map_msg->info.height;
    map_->width_ = map_msg->info.width;
    map_->resolution_ = map_msg->info.resolution;
    map_->p_org_.x = map_msg->info.origin.position.x; 
    map_->p_org_.y = map_msg->info.origin.position.y; 
    map_->p_org_.theta = tf2::getYaw(map_msg->info.origin.orientation);
    map_->p_org_q_ = map_msg->info.origin.orientation;
    if(map_->height_ != map_->data_.size() && map_->width_ != map_->data_.size()) map_->data_.assign(map_->height_, std::vector<int8_t>(map_->width_, 0));
    std::vector<int8_t> data_1d = map_msg->data;
    map_->cvt_1d_to_2d(data_1d);
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Set Map");
}

CmdVel PotentialController::get_cmd_vel(geometry_msgs::msg::Pose2D odom_pose, float lidar_t)
{
    Force total_force = calc_potential_force(odom_pose, lidar_t);
    return force_to_cmd_vel(total_force);
}

CmdVel PotentialController::force_to_cmd_vel(Force force)
{
    float theta_force = atan2f(force.y, force.x);

    CmdVel cmd_vel;
    cmd_vel.linear_vel = force.x;
    cmd_vel.angular_vel = force.y;
    return cmd_vel;
}

Force PotentialController::calc_potential_force(geometry_msgs::msg::Pose2D odom_pose, float lidar_t)
{
    // Force repulsive_force_map = calc_replusive_force_map(odom_x, odom_y);
    // Force repulsive_force_scan = calc_replusive_force_scan(odom_t);
    Force repulsive_force_map = {0.0f, 0.0f}; // TODO: implement map-based repulsive force
    Force repulsive_force_scan = {0.0f, 0.0f}; // TODO: implement scan-based repulsive force
    Force attractive_force = calc_attractive_force(odom_pose.theta, lidar_t);
    return repulsive_force_map + repulsive_force_scan + attractive_force;
}

Force PotentialController::calc_replusive_force_map(float odom_x, float odom_y)
{
    //Laser nearest_obstacle = map_->get_nearest_obstacle(odom_x, odom_y);
    Force repulsive_force;
    if(nearest_obstacle.distance < critical_distance_){
        float force_magnitude = replusive_gain_map_ * (1.0f / nearest_obstacle.distance - 1.0f / critical_distance_) / (nearest_obstacle.distance * nearest_obstacle.distance);
        repulsive_force.x = -force_magnitude * cosf(nearest_obstacle.direction);
        repulsive_force.y = -force_magnitude * sinf(nearest_obstacle.direction);
    } else {
        repulsive_force.x = 0.0f;
        repulsive_force.y = 0.0f;
    }
    return repulsive_force;
}

Force PotentialController::calc_replusive_force_scan(float odom_t)
{
    Laser nearest_obstacle = scan_->get_nearest_obstacle();
    Force repulsive_force;
    if(nearest_obstacle.distance < critical_distance_){
        float force_magnitude = replusive_gain_scan_ * (1.0f / nearest_obstacle.distance - 1.0f / critical_distance_) / (nearest_obstacle.distance * nearest_obstacle.distance);
        repulsive_force.x = -force_magnitude * cosf(nearest_obstacle.direction);
        repulsive_force.y = -force_magnitude * sinf(nearest_obstacle.direction);
    } else {
        repulsive_force.x = 0.0f;
        repulsive_force.y = 0.0f;
    }
    return repulsive_force;
}

Force PotentialController::calc_attractive_force(float odom_t, float lidar_t)
{
    Laser open_laser = scan_->get_open_laser(odom_t, lidar_t);
    Force attractive_force;
    attractive_force.x = attractive_gain_ * open_laser.distance * cosf(open_laser.direction);
    attractive_force.y = attractive_gain_ * open_laser.distance * sinf(open_laser.direction);
    return attractive_force;
}

PotentialController::~PotentialController(){}

} // namespace lsa_nav_controller