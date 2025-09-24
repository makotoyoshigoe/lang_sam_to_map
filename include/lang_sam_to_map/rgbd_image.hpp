// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/rgb_image.hpp"
#include <image_geometry/pinhole_camera_model.h>

namespace lang_sam_to_map{
class RGBDImage : public lang_sam_to_map::RGBImage
{
    public:
    RGBDImage(
        sensor_msgs::msg::Image::ConstSharedPtr color_msg,
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg);
    ~RGBDImage();
    bool uv_to_xyz(int u, int v, cv::Point3d & xyz);
    cv::Mat cv_depth_;
    

    private:
    image_geometry::PinholeCameraModel cam_model_;
};

}