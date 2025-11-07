// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

// #include <rclcpp/rclcpp.hpp>
#include "lang_sam_to_map/sensor/rgbd_image.hpp"

namespace lang_sam_to_map{

RGBDImage::RGBDImage(float min_dehth_th, uint8_t depth_correct_range)
: RGBImage(), DepthImage(min_dehth_th, depth_correct_range)
{
}
RGBDImage::~RGBDImage(){}

}
