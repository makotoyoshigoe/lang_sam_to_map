// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/sensor/depth_image.hpp"

namespace lang_sam_to_map{
DepthImage::DepthImage(
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg)
: Image()
{
    img_msg_to_cv(depth_msg, cv_depth_);
    cam_model_.fromCameraInfo(camera_info_msg);
}

bool DepthImage::uv_to_xyz(int u, int v, cv::Point3d & xyz)
{
    float z = static_cast<float>(cv_depth_.at<uint16_t>(v, u)) * 0.001f;
    if(std::isnan(z)) return false;
    xyz = cam_model_.projectPixelTo3dRay(cv::Point2d(u, v)) * z;
    return true;
}

DepthImage::~DepthImage(){}

}
