// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include "lsa_nav_controller/core/potential_controller.hpp"

namespace lsa_nav_controller
{
PotentialController::PotentialController(
    float critical_distance, 
    float repulsive_gain_map, 
    float repulsive_gain_scan, 
    float attractive_gain, 
    float detect_angle_start,
    float detect_angle_end,
    int detect_angle_division_num, 
    float min_distance,
    float max_linear_vel,
    float max_angular_vel, 
    float sigma):
critical_distance_(critical_distance), 
repulsive_gain_(repulsive_gain_map),
repulsive_gain_map_(repulsive_gain_map), 
repulsive_gain_scan_(repulsive_gain_scan),
min_distance_(min_distance),
max_linear_vel_(max_linear_vel),
max_angular_vel_(max_angular_vel),
attractive_gain_(attractive_gain),
sigma_(sigma)
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
    float range_min = scan_msg->range_min;;
    float range_max = scan_msg->range_max;
    for(size_t i = 0; i < scan_msg->ranges.size(); ++i){
        scan_->ranges_[i] = range_min > scan_msg->ranges[i] || scan_msg->ranges[i] > range_max ? 
                            range_max : scan_msg->ranges[i];
    }
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

CmdVel PotentialController::get_cmd_vel(
    geometry_msgs::msg::Pose2D odom_pose, 
    geometry_msgs::msg::Pose2D lidar_pose)
{
    // Set poses
    scan_->set_lidar_pose(lidar_pose);
    map_->set_odom_pose(odom_pose);

    geometry_msgs::msg::Pose2D odom_lidar_pose;
    odom_lidar_pose.x = odom_pose.x + cosf(odom_pose.theta) * lidar_pose.x - sinf(odom_pose.theta) * lidar_pose.y;
    odom_lidar_pose.y = odom_pose.y + sinf(odom_pose.theta) * lidar_pose.x + cosf(odom_pose.theta) * lidar_pose.y;
    odom_lidar_pose.theta = odom_pose.theta + lidar_pose.theta;
    return force_to_cmd_vel(calc_potential_force(odom_pose));
}

CmdVel PotentialController::force_to_cmd_vel(Force force)
{
    float force_magnitude = hypotf(force.x, force.y); 
    float theta_force = atan2f(force.y, force.x);
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), 
                "Force mag: %.3f, theta: %.3f, Force X: %.3f, Force Y: %.3f", 
                force_magnitude, theta_force, force.x, force.y);

    CmdVel cmd_vel;
    float linear_vel = force_magnitude > 1e-3 ? force.x / force_magnitude * max_linear_vel_ : 0.0f;
    float linear_vel_vec = linear_vel / fabs(linear_vel);
    cmd_vel.linear_vel = fabs(linear_vel) > max_linear_vel_ ? linear_vel_vec * max_linear_vel_ : linear_vel;
    float ang_vel_vec = theta_force / fabs(theta_force);
    cmd_vel.angular_vel = fabs(theta_force) > max_angular_vel_ ? ang_vel_vec * max_angular_vel_ : theta_force;
    return cmd_vel;
}

Force PotentialController::calc_potential_force(
    geometry_msgs::msg::Pose2D odom_pose) 
{
    // Force repulsive_force_map = calc_repulsive_force_map(odom_x, odom_y);
    Force repulsive_force_scan = calc_repulsive_force_scan();
    Force attractive_force = calc_attractive_force();
    //return attractive_force - repulsive_force_map - repulsive_force_scan;
    return attractive_force + repulsive_force_scan;
}

Force PotentialController::calc_repulsive_force_map(float odom_x, float odom_y)
{
    //Laser nearest_obstacle = map_->get_nearest_obstacle(odom_x, odom_y);
    //Force repulsive_force;
    //if(nearest_obstacle.distance < critical_distance_){
    //    float force_magnitude = repulsive_gain_map_ * (1.0f / nearest_obstacle.distance - 1.0f / critical_distance_) / (nearest_obstacle.distance * nearest_obstacle.distance);
    //    repulsive_force.x = -force_magnitude * cosf(nearest_obstacle.direction);
    //    repulsive_force.y = -force_magnitude * sinf(nearest_obstacle.direction);
    //} else {
    //    repulsive_force.x = 0.0f;
    //    repulsive_force.y = 0.0f;
    //}
    //return repulsive_force;
    return {0.0f, 0.0f}; // TODO: implement map-based repulsive force
}

Force PotentialController::calc_repulsive_force_scan(void)
{
    float angle = scan_->angle_min_;
    float angle_increment = scan_->angle_increment_;
    float rx = 0.0f, ry = 0.0f;
    for(auto & range : scan_->ranges_){
        // Values greater than critical distance are ignored
        if(range < critical_distance_){
            // Force vector
            float ex = cos(angle), ey = sin(angle);

            // Force magnitude
            float mag = repulsive_gain_ * exp(-range / sigma_);

            // Accumulate force
            rx += mag * (-ex);
            ry += mag * (-ey);
        }
        // Calculate repulsive force
        angle += angle_increment;
    }
    return {rx, ry};
    // std::vector<LaserXY> exceed_lasers;
    // scan_->get_exceed_threshold_lasers(critical_distance_, exceed_lasers);
    // float fx = 0.0f, fy = 0.0f;
    // for(auto &laser : exceed_lasers){
    //     float x = laser.x;
    //     float y = laser.y;
    //     float r = hypot(x, y);
    //     if(r < min_distance_) r = min_distance_;

    //     float f = (1 / r - 1 / critical_distance_) / r;
    //     nomalize(x, y, x, y);
    //     fx += f * x;
    //     fy += f * y;
    // }
    // return {fx * repulsive_gain_scan_, fy * repulsive_gain_scan_};
}

void PotentialController::nomalize(
    float x_in, float y_in, float & x_out, float & y_out)
{
    float r = hypot(x_in, y_in);
    x_out = 0.0f;
    y_out = 0.0f;
    if(r >= 1e-6){
        x_out = x_in / r;
        y_out = y_in / r;
    }
}

Force PotentialController::calc_attractive_force(void)
{
    Laser open_laser = scan_->get_open_laser();
    float r = open_laser.distance, t = open_laser.direction;
    float fx = r * cosf(t), fy = r * sinf(t);
    return {fx * attractive_gain_, fy * attractive_gain_};
}

PotentialController::~PotentialController(){}

} // namespace lsa_nav_controller