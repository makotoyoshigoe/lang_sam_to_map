// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include "lang_sam_to_map/map/map.hpp"

namespace lang_sam_to_map{

Map::Map(std::string frame_id, float resolution, int width, int height, float map_offset_x, float map_offset_y)
: frame_id_(frame_id), resolution_(resolution), width_(width), height_(height), 
  map_offset_x_(map_offset_x), map_offset_y_(map_offset_y)
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

bool Map::xy_to_index(float x, float y, int & ix, int & iy)
{
	ix = static_cast<int>((x - ox_) / resolution_);
	iy = static_cast<int>((y - oy_) / resolution_);
	if(is_out_range(ix, iy)) return false;
    return true;
}

void Map::get_map_msg(nav_msgs::msg::OccupancyGrid & output)
{
    output.header.frame_id = frame_id_;
    output.info.height = height_;
    output.info.origin.position.x = ox_;
    output.info.origin.position.y = oy_;
    tf2::convert(oq_, output.info.origin.orientation);
    output.info.resolution = resolution_;
    output.info.width = width_;
    cvt_2d_to_1d(output);
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
}

void Map::fill_point(float x, float y, int8_t v)
{
    int ix, iy;
    if(xy_to_index(x, y, ix, iy)) data_[ix][iy] = v;
}

bool Map::is_out_range(int ix, int iy)
{
    return ix < 0 || ix >= width_ || iy < 0 || iy >= height_;
}

Map::~Map(){}

}
