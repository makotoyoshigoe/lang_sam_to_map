// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <cmath>

#include "lsa_nav_controller/sensor/scan.hpp"

namespace lsa_nav_controller
{
Scan::Scan(
    float detect_angle_start, 
    float detect_angle_end, 
    int detect_angle_division_num)
: detect_angle_start_(detect_angle_start),
  detect_angle_end_(detect_angle_end),
  detect_angle_division_num_(detect_angle_division_num)
{
    set_detected_lasers();
}

void Scan::set_detected_lasers(void)
{
    detected_lasers_.clear();
    float division_angle = (detect_angle_end_ - detect_angle_start_) / detect_angle_division_num_;
    for(int i = 0; i < detect_angle_division_num_; ++i){
        float start_angle = detect_angle_start_ + i * division_angle;
        float end_angle = start_angle + division_angle;
        DetectedLaser dl;
        dl.start = start_angle;
        dl.end = end_angle;
        detected_lasers_.emplace_back(dl);
    }
}

Laser Scan::get_open_laser(void)
{
    Laser open_laser;
    open_laser.direction = 0.0f;
    open_laser.distance = 0.0f; // initialize with a value larger than max range

    for(auto detected_laser : detected_lasers_){
        Laser avg_laser = get_laser_average(detected_laser.start, detected_laser.end);
        float distance = avg_laser.distance;
        if(distance > open_laser.distance){
            open_laser.direction = avg_laser.direction;
            open_laser.distance = distance;
        }else if(distance == open_laser.distance){
            // if same distance, choose the one closer to 0 rad
            if(fabsf(avg_laser.direction) < fabsf(open_laser.direction)){
                open_laser.direction = avg_laser.direction;
                open_laser.distance = distance;
            }
        }
    }
    return open_laser;
}

Laser Scan::get_laser_average(float start_angle, float end_angle)
{
    Laser avg_laser;
    avg_laser.direction = (start_angle + end_angle) / 2.0f;
    float sum_distance = 0.0f;
    int count = 0;
    size_t is = rad_to_index(start_angle);
    size_t ie = rad_to_index(end_angle);

    for(size_t i = is; i <= ie; ++i){
        float distance = ranges_[i];
        if(distance < range_min_ || distance > range_max_) distance = range_max_;
        sum_distance += distance;
        count++;
    }
    avg_laser.distance = sum_distance / count;
    return avg_laser;
}

void Scan::set_lidar_pose(
    geometry_msgs::msg::Pose2D lidar_pose){lidar_pose_ = lidar_pose;}

size_t Scan::rad_to_index(float rad)
{
    if(rad < angle_min_ || rad > angle_max_) return static_cast<size_t>(-1);
    return static_cast<size_t>((rad - angle_min_) / angle_increment_);
}

void Scan::get_exceed_threshold_lasers(
    float threshold, 
    std::vector<LaserXY> & exceed_lasers) 
{
    exceed_lasers.clear();
    for(size_t i = 0; i < ranges_.size(); ++i){
        float distance = ranges_[i];
        if(distance > threshold){
            float t = angle_min_ + i * angle_increment_;
            float r = distance;
            LaserXY laser;
            laser.x = distance * cosf(t);
            laser.y = distance * sinf(t);
            exceed_lasers.emplace_back(cvt_lidar_to_robot(laser));
        }
    }
}

LaserXY Scan::cvt_lidar_to_robot(LaserXY laser)
{
    float rx = laser.x;
    float ry = laser.y;
    float t = lidar_pose_.theta;
    float x = lidar_pose_.x + cosf(t) * rx - sinf(t) * ry;
    float y = lidar_pose_.y + sinf(t) * rx + cosf(t) * ry;
    return {x, y};
}
Scan::~Scan(){}

} // namespace lsa_nav_controller