// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/image.hpp"
#include <image_geometry/pinhole_camera_model.h>

namespace lang_sam_to_map{
class DepthImage : public Image{
    public:
    DepthImage(
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg);
    ~DepthImage();
    bool uv_to_xyz(int u, int v, cv::Point3d & xyz);

    private:
    cv::Mat cv_depth_;
    image_geometry::PinholeCameraModel cam_model_;
};

} // namespace lang_sam_to_map
