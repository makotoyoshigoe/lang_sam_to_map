// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include<opencv2/opencv.hpp>
#include<sensor_msgs/msg/image.hpp>

namespace lang_sam_to_map{

bool img_msg_to_cv(
    sensor_msgs::msg::Image::ConstSharedPtr img_msg,
    cv::Mat& rgb_image);

} // namespace lang_sam_to_map
