// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/time.hpp>
#include "lang_sam_to_map/sensor/rgbd_image.hpp"
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/voxel_grid.h>

namespace lang_sam_to_map{
class RGBDPointcloudConverter{
    public:
    RGBDPointcloudConverter(
        sensor_msgs::msg::Image::ConstSharedPtr color_msg,
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);
    RGBDPointcloudConverter(
        float vg_leaf_size, float max_depth_th, float min_depth_th);
    ~RGBDPointcloudConverter();
    void update_images_info(
        sensor_msgs::msg::Image::ConstSharedPtr color_msg, 
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);
    void create_point_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output);
    void down_sampling(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output);
    sensor_msgs::msg::PointCloud2 pcl_to_msg(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);

    private:
    std::string rgb_camera_frame_id_;
    std::unique_ptr<RGBDImage> rgbd_image_;
    float max_depth_th_, min_depth_th_;
    pcl::VoxelGrid<pcl::PointXYZRGB>::Ptr voxel_grid_filter_;
};
}
