// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lsa_nav_controller/sensor/scan.hpp"
#include "lsa_nav_controller/map/map.hpp"

namespace lsa_nav_controller
{
struct CmdVel{
    float linear_vel;
    float angular_vel;
};

struct Force{
    float x = 0.0f;
    float y = 0.0f;
    Force operator+(const Force & other) const
    {
        return {x + other.x, y + other.y};
    }
    Force operator-(const Force & other) const
    {
        return {x - other.x, y - other.y};
    }
};

class PotentialController{
    public:
    // constructor/destructor
    PotentialController(
        float critical_distance, 
        float repulsive_gain_map, 
        float repulsive_gain_scan, 
        float attractive_gain, 
        float min_distance, 
        float max_linear_vel,
        float max_angular_vel, 
        float sigma);
    ~PotentialController();

    // setters
    void set_scan(Scan & scan);
    void set_map_data(
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg);
    
    // main methods
    CmdVel get_cmd_vel(
        geometry_msgs::msg::Pose2D odom_pose, 
        geometry_msgs::msg::Pose2D lidar_pose, 
        std::array<float, 3> open_laser_info);
    CmdVel force_to_cmd_vel(Force force);
    Force calc_potential_force(void);
    Force calc_repulsive_force_map(float odom_x, float odom_y);
    Force calc_repulsive_force_scan(void);
    Force calc_attractive_force(void);
    void nomalize(float x_in, float y_in, float & x_out, float & y_out);

    private:
    Scan scan_;
    std::unique_ptr<Map> map_;

    // parameters
    float critical_distance_;
    float repulsive_gain_;
    float sigma_;
    float repulsive_gain_map_;
    float repulsive_gain_scan_;
    float attractive_gain_;
    float detect_angle_start_;
    float detect_angle_end_;
    float min_distance_; // for preventing division near zero
    float max_linear_vel_;
    float max_angular_vel_;
    int detect_angle_division_num_;
    float robot_radius_;

    // [0]: direction, [1]: distance, [2]: ratio score
    std::array<float, 3> open_laser_info_;
};

} // namespace lsa_nav_controller