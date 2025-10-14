// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

namespace lsa_nav_controller
{
class Map{
    public:
    Map(void);
    ~Map();
    void update_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);
    void fill_bottom(void);
    void cvt_1d_to_2d(std::vector<int8_t> & data);
    void get_map_msg(nav_msgs::msg::OccupancyGrid & output);
    void cvt_2d_to_1d(std::vector<int8_t> & data);

    protected:
    std::string frame_id_;
    geometry_msgs::msg::Pose2D p_org_;
    int width_, height_;
    float resolution_;
    std::vector<std::vector<int8_t>> data_;
};
    
} // namespace lsa_nav_controller
