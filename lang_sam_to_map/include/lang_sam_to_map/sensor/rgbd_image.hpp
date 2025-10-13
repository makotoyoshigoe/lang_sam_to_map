// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/sensor/rgb_image.hpp"
#include "lang_sam_to_map/sensor/depth_image.hpp"

namespace lang_sam_to_map{
class RGBDImage : public lang_sam_to_map::RGBImage, public lang_sam_to_map::DepthImage
{
    public:
    RGBDImage(
        sensor_msgs::msg::Image::ConstSharedPtr color_msg,
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg);
    RGBDImage(void);
    ~RGBDImage();
    
    private:
};

}
