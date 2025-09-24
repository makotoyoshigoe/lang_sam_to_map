// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include<cv_bridge/cv_bridge.h>

namespace lang_sam_to_map{
class RGBImage
{
    public:
    RGBImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
    ~RGBImage();
    bool img_msg_to_cv(
        sensor_msgs::msg::Image::ConstSharedPtr img_msg,
        cv::Mat & cv_image);
    bool cv_to_msg(
        const std_msgs::msg::Header & header, 
        const cv::Mat & input_img,
        sensor_msgs::msg::Image & msg);
    cv::Mat cv_color_;
    cv::Vec3b get_pixel_color(int u, int v);
    void get_image_size(int & rows, int & cols);

    private:
};

}