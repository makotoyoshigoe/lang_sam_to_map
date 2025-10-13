// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/sensor/depth_image.hpp"

namespace lang_sam_to_map{
DepthImage::DepthImage(
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg)
: Image()
{
    set_depth_image_from_msg(depth_msg);
    set_camera_model_from_msg(camera_info_msg);
}

DepthImage::DepthImage(void)
: Image()
{
    
}

bool DepthImage::uv_to_xyz(int u, int v, cv::Point3d & xyz)
{
    float z = static_cast<float>(cv_depth_.at<uint16_t>(v, u)) * 0.001f;
    if(std::isnan(z)) return false;
    xyz = cam_model_.projectPixelTo3dRay(cv::Point2d(u, v)) * z;
    return true;
}

void DepthImage::set_depth_image_from_msg(
    sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    img_msg_to_cv(msg, cv_depth_);
}

void DepthImage::set_camera_model_from_msg(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
    cam_model_.fromCameraInfo(msg);
}

DepthImage::~DepthImage(){}

}
