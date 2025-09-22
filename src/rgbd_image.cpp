// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include "lang_sam_to_map/rgbd_image.hpp"

namespace lang_sam_to_map{

RGBDImage::RGBDImage(
    sensor_msgs::msg::Image::ConstSharedPtr color_msg,
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg)
: RGBImage(color_msg)
{
    this->img_msg_to_cv(depth_msg, cv_depth_);
}



RGBDImage::~RGBDImage(){}

}