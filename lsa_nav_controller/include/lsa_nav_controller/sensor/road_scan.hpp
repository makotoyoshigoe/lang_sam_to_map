// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <sensor_msgs/msg/laser_scan.hpp>

namespace lsa_nav_controller
{
class RoadScan{
    public:
    RoadScan(float max_angle_abs, float min_angle_abs, float angle_increment, float max_range, float min_range);
    ~RoadScan();
    void get_road_scan(
        std::vector<float> ranges, sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    
    private:
    float max_angle_abs_, min_angle_abs_, angle_increment_;
    float max_range_, min_range_;
};
    
} // namespace lsa_nav_controller
