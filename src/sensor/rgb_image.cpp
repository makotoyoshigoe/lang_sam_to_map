// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include "lang_sam_to_map/sensor/rgb_image.hpp"

namespace lang_sam_to_map{

RGBImage::RGBImage(void) : Image(){}

RGBImage::RGBImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
: Image()
{
    set_color_image_from_msg(msg);
}

cv::Vec3b RGBImage::get_pixel_color(int u, int v)
{
    return cv_color_.at<cv::Vec3b>(v, u);
}

void RGBImage::get_image_size(int & rows, int & cols)
{
    rows = cv_color_.rows;
    cols = cv_color_.cols;
}

void RGBImage::set_color_image_from_msg(
        sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    img_msg_to_cv(msg, cv_color_);
    cv::cvtColor(cv_color_, cv_color_, CV_BGR2RGB);
}

RGBImage::~RGBImage(){}

}
