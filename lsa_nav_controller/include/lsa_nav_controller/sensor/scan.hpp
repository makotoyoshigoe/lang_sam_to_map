// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace lsa_nav_controller
{
class Scan{
    public:
    Scan(void);
    ~Scan();

    // methods
    void set_lidar_pose(geometry_msgs::msg::Pose2D lidar_pose);
    size_t rad_to_index(float rad);
    void set_scan_data(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    // public member variables
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    float range_min_;
    float range_max_;
    std::vector<float> ranges_;
    std::string frame_id_;

    private:
    float detect_angle_start_; 
    float detect_angle_end_; 
    int detect_angle_division_num_;
    geometry_msgs::msg::Pose2D lidar_pose_;
};

} // namespace lsa_nav_controller