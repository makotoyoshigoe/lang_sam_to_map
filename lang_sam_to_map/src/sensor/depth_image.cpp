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
    //if (z <= 0) z = correct_depth_loss(u, v) * 0.001f;
    xyz = cam_model_.projectPixelTo3dRay(cv::Point2d(u, v)) * z;
    return true;
}

int DepthImage::correct_depth_loss(int u, int v)
{
    int n = 0, sum_depth = 0;
    for(int i=-1; i<=1; ++i){
        for(int j=-1; j<=1; ++j){
            if(v+i < 0 || v+i >= cv_depth_.rows || u+j < 0 || u+j >= cv_depth_.cols) continue;
            int depth = cv_depth_.at<uint16_t>(v+i, u+j);
            if(depth <= 0 && !std::isnan(depth)) continue;
            sum_depth += depth;
            n++;
        }
    }
    return n != 0 ? sum_depth / n : 0;
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
