// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include "lang_sam_to_map/core/lsa_map_generator.hpp"


namespace lang_sam_to_map
{

LSAMapGenerator::LSAMapGenerator(
    std::vector<sensor_msgs::msg::Image> & masks_msg_vec, 
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg, 
    std::string frame_id, float resolution, int width, int height)
: Map(frame_id, resolution, width, height)
{
    mask_images_.reset(new MaskImages(masks_msg_vec));
    depth_image_.reset(new DepthImage(depth_msg, camera_info_msg));
}

void LSAMapGenerator::create_grid_map_from_contours(tf2::Transform & tf_camera_to_odom)
{
    // Reset Map
    reset_map();

	RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Started to create map");

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> occupied_pc_vec;
    contours_to_3d_point(occupied_pc_vec);

    pcl::PointCloud<pcl::PointXYZ>::Ptr unoccupied_pc(new pcl::PointCloud<pcl::PointXYZ>);
    find_unoccupied_3d_point(unoccupied_pc);

	RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Completed to create pointcloud");

    // Transform coordinate camera frame->odom
    for(auto &pc: occupied_pc_vec){
        pcl_ros::transformPointCloud(*pc, *pc, tf_camera_to_odom);
    }
	RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Transform Pointcloud");
    // pcl_ros::transformPointCloud(*unoccupied_pc, *unoccupied_pc, tf_camera_to_odom);

    // Point xy -> Grid xy
	pcl::PointCloud<pcl::PointXYZ>::iterator pt;
    // for (pt=unoccupied_pc->points.begin(); pt < unoccupied_pc->points.end()-1; pt++){
    //     int index = xy_to_index((*pt).x, (*pt).y);
    //     if(index == -1) continue;
    //     lang_sam_map_.data[index] = 0;
    // }
	for(auto &pc: occupied_pc_vec){
 		for (pt=pc->points.begin(); pt < pc->points.end(); pt++){
            bresenham(
                static_cast<int>(((*pt).x - ox_) / resolution_), 
                static_cast<int>(((*pt).y - oy_) / resolution_));
            int ix, iy;
            if(!xy_to_index((*pt).x, (*pt).y, ix, iy)) continue;
            // RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "ix, iy: %d, %d", ix, iy);
            data_[ix][iy] = 100;
            // fill_point((*pt).x, (*pt).y, 100);
		}
	}
    
	RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Completed to create map");
    // double t = tf2::getYaw(lang_sam_map_.info.origin.orientation);
    // RCLCPP_INFO(get_logger(), "Map Origin: x: %lf, y: %lf, t: %lf", 
    //     lang_sam_map_.info.origin.position.x, lang_sam_map_.info.origin.position.y, t);
}

void LSAMapGenerator::contours_to_3d_point(
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & occupied_pc_vec)
{
    std::vector<std::vector<cv::Point>> contours;
    mask_images_->find_contours(contours);
    // contours points and depth -> pcl
	for(size_t i=0; i<contours.size(); ++i){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pointcloud->points.reserve(contours[i].size());
        for(auto &p: contours[i]){
            cv::Point3d xyz;
            bool cvt_res = depth_image_->uv_to_xyz(p.x, p.y, xyz);
            if(!cvt_res) continue;
            pcl::PointXYZ p_xyz(xyz.x, xyz.y, xyz.z);
            pointcloud->points.emplace_back(p_xyz);
            
		}
        occupied_pc_vec.emplace_back(pointcloud);
	}
}

void LSAMapGenerator::find_unoccupied_3d_point(
    pcl::PointCloud<pcl::PointXYZ>::Ptr & unoccupied_pc)
{
    cv::Mat bin_mask;
    mask_images_->get_bin_mask(bin_mask);
    unoccupied_pc->points.reserve(bin_mask.rows*bin_mask.cols);
    bin_mask.forEach<uchar>([&](uchar &pixel, const int position[]) -> void {
        if (pixel > 0) {
            cv::Point3d xyz;
            bool cvt_res = depth_image_->uv_to_xyz(position[1], position[0], xyz);
            if(cvt_res){
                pcl::PointXYZ p_xyz(xyz.x, xyz.y, xyz.z);
                unoccupied_pc->points.emplace_back(p_xyz);
            }
        }
    });
}

void LSAMapGenerator::bresenham(int x_e, int y_e)
{
    int x0 = static_cast<int>(width_ / 2);
    int y0 = static_cast<int>(height_ / 2);
    // int x0, y0;
    // if(!xy_to_index(ox_, oy_, x0, y0)) return;
	// RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "(x0, y0) = (%d, %d)", x0, y0);

    // RCLCPP_INFO(get_logger(), "(x0: %d, y0: %d)", x0, y0);
    int x1 = x_e;
    int y1 = y_e;
    // RCLCPP_INFO(get_logger(), "(x1: %d, y1: %d)", x1, y1);

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true) {
        if(is_out_range(x0, y0)) break;
        if (x0 == x1 && y0 == y1) break;

        if(data_[x0][y0] != 100) data_[x0][y0] = 0;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

cv::Mat LSAMapGenerator::get_visalize_image(
    sensor_msgs::msg::Image::ConstSharedPtr base, 
    std::vector<sensor_msgs::msg::RegionOfInterest> & boxes)
{
    return mask_images_->vis_mask_contours_bbox(base, boxes);
}

LSAMapGenerator::~LSAMapGenerator(){}
    
} // namespace lang_sam_to_map
