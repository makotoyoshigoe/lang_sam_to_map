// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/rgbd_pointcloud_converter.hpp"
#include <pcl/filters/voxel_grid.h>

namespace lang_sam_to_map
{
RGBDPointcloudConverter::RGBDPointcloudConverter(
    sensor_msgs::msg::Image::ConstSharedPtr color_msg,
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
{
    rgbd_image_.reset(new RGBDImage(color_msg, depth_msg, camera_info));
}

void RGBDPointcloudConverter::create_point_cloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output, 
    float max_depth_th, float min_depth_th)
{
    // Reserve PointCloud Vector
    int rows, cols;
    rgbd_image_->get_image_size(rows, cols);
    output->points.reserve(rows * cols);
    
    RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Before Create Pointcloud");

    // Create PointCloud
    for (int v = 0; v < rows; ++v){
        for (int u = 0; u < cols; ++u) {
            cv::Point3d xyz;
            bool cvt_res = rgbd_image_->uv_to_xyz(u, v, xyz);
			if(!cvt_res || xyz.z > max_depth_th || xyz.z < min_depth_th) continue;
            cv::Vec3b color = rgbd_image_->get_pixel_color(u, v);
            pcl::PointXYZRGB p(
                xyz.x, xyz.y, xyz.z, color[2], color[1], color[0]);
            output->points.emplace_back(p);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Before Set Cloud Info");
    // Set PointCloud Infomation
    output->width = output->points.size();
	output->height = 1;
	output->is_dense = false;
    RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "After Set Cloud Info");
}

void RGBDPointcloudConverter::down_sampling(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & output, float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZRGB>::Ptr voxel_grid_filter(new pcl::VoxelGrid<pcl::PointXYZRGB>);
    voxel_grid_filter->setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid_filter->setInputCloud(input);
    voxel_grid_filter->filter(*output);
}

RGBDPointcloudConverter::~RGBDPointcloudConverter(){}

}