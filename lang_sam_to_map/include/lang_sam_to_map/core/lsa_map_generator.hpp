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
        std::string frame_id, float resolution, 
        float max_valid_th, float min_valid_th, 
        float noise_contour_area_th, float connect_grid_th, 
        float map_offset_x, float map_offset_y,
        int outer_frame_th);
    ~LSAMapGenerator();
    void set_origin(float ox, float oy, geometry_msgs::msg::Quaternion oq);
    void update_image_infos(
        std::vector<sensor_msgs::msg::Image> & masks_msg_vec, 
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg);
    bool create_grid_map_from_contours(
        const tf2::Transform & tf_camera_to_base);
    void contours_to_3d_point(void);
    bool is_outer_frame(cv::Point & p);
    void connect_grids(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & pc_vec);
    void plot_grids_and_raycast(
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & pc_vec,
        uint8_t value);
    void bresenham(int x_s, int y_s, int x_e, int y_e);
    void bresenham_fill(int x_e, int y_e);
    bool get_visualize_msg(
        bool raw, 
        sensor_msgs::msg::Image & output, 
        sensor_msgs::msg::Image::ConstSharedPtr base, 
        std::vector<sensor_msgs::msg::RegionOfInterest> & boxes);

    private:
    std::vector<sensor_msgs::msg::Image> masks_msg_vec_;
    std::unique_ptr<MaskImages> mask_images_;
    std::unique_ptr<DepthImage> depth_image_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> occupied_pc_vec_, outer_pc_vec_;
    float max_valid_th_, min_valid_th_, connect_grid_th_;
    std::vector<Grid> occupied_grid_;
    int outer_frame_th_;
};

} // namespace lang_sam_to_map

