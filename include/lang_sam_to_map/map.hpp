// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace lang_sam_to_map{
class Map
{
    public:
        Map(std::string frame_id, float resolution, int width, int height);
        ~Map();
        float get_resolution(void);
        float get_one_side_size(void);
		float get_map_size(void);
        void get_data(std::vector<uint8_t> & data);
        void set_origin(float ox, float oy);
		bool xy_to_index(float x, float y, int & ix, int & iy);
        void reset_map(void);
        nav_msgs::msg::OccupancyGrid get_map_msg(void);
        void cvt_2d_to_1d(nav_msgs::msg::OccupancyGrid & msg);
        void fill_bottom(void);
        void fill_point(float x, float y, int8_t v);
        bool is_out_range(int ix, int iy);

    protected:
		std::string frame_id_;
		float resolution_;
		int width_, height_;
		float ox_, oy_;
		std::vector<std::vector<int8_t>> data_;
};

}
