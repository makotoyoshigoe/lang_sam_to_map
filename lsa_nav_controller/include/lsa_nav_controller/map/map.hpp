// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

namespace lsa_nav_controller
{
struct Grid{
    int x;
    int y;
    Grid operator-(const Grid & other) const
    {
        return {x - other.x, y - other.y};
    }
};

class Map{
    public:
    Map(void);
    ~Map();
    void update_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);
    void fill_bottom(void);
    void cvt_1d_to_2d(std::vector<int8_t> & data);
    void get_map_msg(nav_msgs::msg::OccupancyGrid & output);
    void get_map_wor_msg(nav_msgs::msg::OccupancyGrid & output); // 原点の回転を0にした地図を取得
    void cvt_2d_to_1d(std::vector<int8_t> & data);
    std::string get_map_frame_id(void);
    bool is_out_range(Grid grid);
    Grid point_to_grid(float x, float y);
    void set_odom_pose(geometry_msgs::msg::Pose2D odom_pose);
    
    // member variables
    std::string frame_id_;
    geometry_msgs::msg::Pose2D p_org_;
    uint32_t width_, height_;
    float resolution_;
    std::vector<std::vector<int8_t>> data_;
    bool init_map_receive_;
    geometry_msgs::msg::Quaternion p_org_q_;
    geometry_msgs::msg::Pose2D odom_pose_;
};
    
} // namespace lsa_nav_controller
