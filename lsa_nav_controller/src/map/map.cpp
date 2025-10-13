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

void Map::update_map(nav_msgs::msg::OccupancyGrid map)
{
    height_ = map.info.height;
    width_ = map.info.width;
    resolution_ = map.info.resolution;
    p_org_.x = map.info.origin.position.x; 
    p_org_.y = map.info.origin.position.y; 
    p_org_.theta = tf2::getYaw(map.info.origin.orientation);
    if(height_ != data_.size() && width_ != data_.size()) data_.assign(height_, std::vector<int8_t>(width_, 0));
    cvt_1d_to_2d(map.data);
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "Read Map");
}

void Map::cvt_1d_to_2d(std::vector<int8_t> & data)
{
    for(int y = 0; y < height_; ++y){
        for(int x = 0; x < width_; ++x){
            data_[y][x] = data[y*height_ + width_];
        }
    }
}

Map::~Map(){}

} // namespace lsa_nav_controller
