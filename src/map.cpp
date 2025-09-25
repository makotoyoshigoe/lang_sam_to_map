// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include "lang_sam_to_map/map.hpp"

namespace lang_sam_to_map{

Map::Map(std::string frame_id, float resolution, int width, int height)
: frame_id_(frame_id), resolution_(resolution), width_(width), height_(height)
{
    reset_map();
}

float Map::get_resolution()
{
	return resolution_;
}

float Map::get_one_side_size()
{
	return height_;
}

float Map::get_map_size()
{
	return width_ * height_;
}

void Map::set_origin(float ox, float oy)
{
	ox_ = ox;
	oy_ = oy;
}

void Map::reset_map(void)
{
    data_.assign(height_, std::vector<int8_t>(width_, -1));
}

void Map::xy_to_index(float x, float y, int & ix, int & iy)
{
	ix = static_cast<int>((x - ox_) / resolution_);
	iy = static_cast<int>((y - oy_) / resolution_);
	if(ix < 0 || ix > width_ || iy < 0 || iy > height_){
        ix = -1;iy = -1;
    }
}

nav_msgs::msg::OccupancyGrid Map::get_map_msg(rclcpp::Time stamp)
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = stamp;
    msg.info.height = height_;
    msg.info.origin.position.x = ox_;
    msg.info.origin.position.y = oy_;
    msg.info.resolution = resolution_;
    msg.info.width = width_;
    // RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Before convert msg");
    cvt_2d_to_1d(msg);
    // RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "After convert msg");
    return msg;
}

void Map::cvt_2d_to_1d(nav_msgs::msg::OccupancyGrid & msg)
{
    msg.data.reserve(width_*height_);
    for(int i=0; i<height_; ++i){
        for(int j=0; j<width_; ++j){
            msg.data.emplace_back(data_[j][i]);
        }
    }
}

void Map::fill_bottom(void)
{
    for(int i=0; i<width_; ++i) data_[i][0] = 100;
    // RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Fill");
}

Map::~Map(){}

}
