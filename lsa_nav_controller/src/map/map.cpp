// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include "lsa_nav_controller/map/map.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

namespace lsa_nav_controller
{

Map::Map(void)
{

}

void Map::update_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map)
{
    nav_msgs::msg::OccupancyGrid msg = *map;
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "%d", msg.info.width);
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Update Map");
    frame_id_ = msg.header.frame_id;
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "After Set Frame Id");
    height_ = msg.info.height;
    width_ = msg.info.width;
    resolution_ = msg.info.resolution;
    p_org_.x = msg.info.origin.position.x; 
    p_org_.y = msg.info.origin.position.y; 
    p_org_.theta = tf2::getYaw(msg.info.origin.orientation);
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Before Assign");
    if(height_ != data_.size() && width_ != data_.size()) data_.assign(height_, std::vector<int8_t>(width_, 0));
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Before Read Map");
    cvt_1d_to_2d(msg.data);
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Read Map");
}

void Map::fill_bottom(void)
{
    for(int i=0; i<width_; ++i) data_[i][0] = 0;
}

void Map::get_map_msg(nav_msgs::msg::OccupancyGrid & output)
{
    output.header.frame_id = frame_id_;
    output.info.height = height_;
    output.info.origin.position.x = p_org_.x;
    output.info.origin.position.y = p_org_.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, p_org_.theta);
    tf2::convert(q, output.info.origin.orientation);
    output.info.resolution = resolution_;
    output.info.width = width_;
    cvt_2d_to_1d(output.data);
}

void Map::cvt_2d_to_1d(std::vector<int8_t> & data)
{
    data.reserve(width_*height_);
    for(int i=0; i<height_; ++i){
        for(int j=0; j<width_; ++j){
            data.emplace_back(data_[j][i]);
        }
    }
}

void Map::cvt_1d_to_2d(std::vector<int8_t> & data)
{
    for(int y = 0; y < height_; ++y){
        for(int x = 0; x < width_; ++x){
            data_[x][y] = data[y*width_+ x];
        }
    }
}

Map::~Map(){}

} // namespace lsa_nav_controller
