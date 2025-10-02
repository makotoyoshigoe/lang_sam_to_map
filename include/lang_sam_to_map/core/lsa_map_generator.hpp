// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/map/map.hpp"
#include "lang_sam_to_map/sensor/depth_image.hpp"
#include "lang_sam_to_map/sensor/mask_images.hpp"
#include <pcl_ros/transforms.hpp>

namespace lang_sam_to_map{
class LSAMapGenerator : public lang_sam_to_map::Map{
    public:
    LSAMapGenerator(
        std::vector<sensor_msgs::msg::Image> & masks_msg_vec, 
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg, 
        std::string frame_id, float resolution, int width, int height, 
        float max_valid_th, float min_valid_th);
    LSAMapGenerator(
        std::string frame_id, float resolution, int width, int height, 
        float max_valid_th, float min_valid_th, float noise_contour_th);
    ~LSAMapGenerator();
    void set_origin(float ox, float oy);
    void update_image_infos(
        std::vector<sensor_msgs::msg::Image> & masks_msg_vec, 
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg);
    bool create_grid_map_from_contours(const tf2::Transform & tf_camera_to_odom);
    void contours_to_3d_point(
        // std::vector<std::vector<std::vector<cv::Point>>> & contours,
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & occupied_pc_vec);
    void plot_occupied_and_raycast(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & occupied_pc_vec);
    void bresenham(int x_e, int y_e);
    bool get_visualize_msg(
        bool raw, 
        sensor_msgs::msg::Image & output, 
        sensor_msgs::msg::Image::ConstSharedPtr base, 
        std::vector<sensor_msgs::msg::RegionOfInterest> & boxes);

    private:
    std::vector<sensor_msgs::msg::Image> masks_msg_vec_;
    std::unique_ptr<MaskImages> mask_images_;
    std::unique_ptr<DepthImage> depth_image_;
    float max_valid_th_, min_valid_th_;
};

} // namespace lang_sam_to_map

