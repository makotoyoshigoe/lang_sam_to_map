// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include<cv_bridge/cv_bridge.h>

#pragma once

namespace lang_sam_to_map
{
class Image{
    public:
    Image();
    ~Image();
    bool img_msg_to_cv(
        sensor_msgs::msg::Image::ConstSharedPtr img_msg,
        cv::Mat & cv_image);
    bool cv_to_msg(
        const std_msgs::msg::Header & header, 
        const cv::Mat & input_img,
        sensor_msgs::msg::Image & msg);
};
    
} // namespace lang_sam_to_map

