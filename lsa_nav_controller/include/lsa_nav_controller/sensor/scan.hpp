// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace lsa_nav_controller
{
struct Laser
{
    float direction; // in radians
    float distance;
};

struct DetectedLaser
{
    float start; // in radians
    float end;
};

struct LaserXY
{
    float x;
    float y;
};

class Scan{
    public:
    Scan(float detect_angle_start, float detect_angle_end, 
        int detect_angle_division_num);
    ~Scan();

    // methods
    void set_detected_lasers(void);
    Laser get_open_laser(void);
    Laser get_laser_average(float start_angle, float end_angle);
    void set_lidar_pose(geometry_msgs::msg::Pose2D lidar_pose);
    size_t rad_to_index(float rad);
    void get_exceed_threshold_lasers(float threshold, 
        std::vector<LaserXY> & exceed_lasers);
    LaserXY cvt_lidar_to_robot(LaserXY laser);

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
    std::vector<DetectedLaser> detected_lasers_;
    geometry_msgs::msg::Pose2D lidar_pose_;
};

} // namespace lsa_nav_controller