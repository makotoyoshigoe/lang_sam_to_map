// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/sensor/image.hpp"
#include <image_geometry/pinhole_camera_model.h>

namespace lang_sam_to_map{
class DepthImage : public Image{
    public:
    DepthImage(float min_depth_th, uint8_t depth_correct_range);
    ~DepthImage();
    bool uv_to_xyz(int u, int v, cv::Point3d & xyz);
    void set_depth_image_from_msg(
        sensor_msgs::msg::Image::ConstSharedPtr msg);
    void set_camera_model_from_msg(
        sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    int correct_depth_loss(int u, int v);

    private:
    float min_depth_th_;
    uint8_t depth_correct_range_;
    cv::Mat cv_depth_;
    image_geometry::PinholeCameraModel cam_model_;
};

} // namespace lang_sam_to_map
