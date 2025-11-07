// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/sensor/depth_image.hpp"

namespace lang_sam_to_map{
DepthImage::DepthImage(float min_depth_th, uint8_t depth_correct_range)
: Image(), min_depth_th_(min_depth_th), depth_correct_range_(depth_correct_range){}

bool DepthImage::uv_to_xyz(int u, int v, cv::Point3d & xyz)
{
    float z = static_cast<float>(cv_depth_.at<uint16_t>(v, u)) * 0.001f;
    if(std::isnan(z)) return false;
    if (z < min_depth_th_) z = correct_depth_loss(u, v) * 0.001f;
    xyz = cam_model_.projectPixelTo3dRay(cv::Point2d(u, v)) * z;
    return true;
}

int DepthImage::correct_depth_loss(int u, int v)
{
    int n = 0, sum_depth = 0;
    uint8_t d = depth_correct_range_;
    for(int i=-d; i<=d; ++i){
        for(int j=-d; j<=d; ++j){
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
