// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>

#include "lsa_nav_controller/sensor/scan.hpp"
#include "lsa_nav_controller/map/map.hpp"

namespace lsa_nav_controller
{
struct CmdVel{
    float linear_vel;
    float angular_vel;
};

struct Force{
    float x;
    float y;
    Force operator+(const Force & other) const
    {
        return {x + other.x, y + other.y};
    }
};

class PotentialController{
    public:
    // constructor/destructor
    PotentialController(
        float critical_distance, 
        float replusive_gain_map, 
        float replusive_gain_scan, 
        float attractive_gain, 
        float detect_angle_start,
        float detect_angle_end,
        int detect_angle_division_num);
    ~PotentialController();

    // setters
    void set_scan_data(
        sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void set_map_data(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg);
    
    // main methods
    CmdVel get_cmd_vel(geometry_msgs::msg::Pose2D odom_pose, float lidar_pose);
    CmdVel force_to_cmd_vel(Force force);
    Force calc_potential_force(geometry_msgs::msg::Pose2D odom_pose, float lidar_pose);
    Force calc_replusive_force_map(float odom_x, float odom_y);
    Force calc_replusive_force_scan(float odom_t);
    Force calc_attractive_force(float odom_t, float lidar_pose); 

    private:
    std::unique_ptr<Scan> scan_;
    std::unique_ptr<Map> map_;

    // parameters
    float critical_distance_;
    float replusive_gain_map_;
    float replusive_gain_scan_;
    float attractive_gain_;
    float detect_angle_start_;
    float detect_angle_end_;
    int detect_angle_division_num_;
};

} // namespace lsa_nav_controller