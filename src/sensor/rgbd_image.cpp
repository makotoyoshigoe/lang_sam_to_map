// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

// #include <rclcpp/rclcpp.hpp>
#include "lang_sam_to_map/sensor/rgbd_image.hpp"

namespace lang_sam_to_map{

RGBDImage::RGBDImage(
    sensor_msgs::msg::Image::ConstSharedPtr color_msg,
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg)
: RGBImage(color_msg), DepthImage(depth_msg, camera_info_msg)
{
    //this->img_msg_to_cv(depth_msg, cv_depth_);
    //cam_model_.fromCameraInfo(camera_info_msg);
}

//RGBDImage::RGBDImage(
//    sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
//    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg)
//: RGBImage()
//{
//    this->img_msg_to_cv(depth_msg, cv_depth_);
//    cam_model_.fromCameraInfo(camera_info_msg);
//}

//bool RGBDImage::uv_to_xyz(int u, int v, cv::Point3d & xyz)
//{
//    float z = static_cast<float>(cv_depth_.at<uint16_t>(v, u)) * 0.001f;
//    if(std::isnan(z)) return false;
//    xyz = cam_model_.projectPixelTo3dRay(cv::Point2d(u, v)) * z;
//    return true;
//}

RGBDImage::~RGBDImage(){}

}
