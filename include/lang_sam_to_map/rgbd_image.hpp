// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/rgb_image.hpp"

namespace lang_sam_to_map{
class RGBDImage : public lang_sam_to_map::RGBImage
{
    public:
    RGBDImage(
        sensor_msgs::msg::Image::ConstSharedPtr color_msg,
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg);
    ~RGBDImage();
    bool uv_to_xyz(
        image_geometry::PinholeCameraModel& cam_model,
        cv::Mat& cv_depth, 
        int u, int v, cv::Point3d& xyz);

    private:
    cv::Mat cv_depth_;
};

}