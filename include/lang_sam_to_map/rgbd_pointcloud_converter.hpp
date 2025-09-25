// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/rgbd_image.hpp"
#include <pcl_ros/transforms.hpp>

namespace lang_sam_to_map{
class RGBDPointcloudConverter{
    public:
    RGBDPointcloudConverter(
        sensor_msgs::msg::Image::ConstSharedPtr color_msg,
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);
    ~RGBDPointcloudConverter();
    void create_point_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output, 
        float max_depth_th, float min_depth_th);
    void down_sampling(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output, float leaf_size);

    private:
    std::unique_ptr<RGBDImage> rgbd_image_;
};
}
