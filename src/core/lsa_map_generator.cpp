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
    std::string frame_id, float resolution, int width, int height, 
    float max_valid_th, float min_valid_th)
: Map(frame_id, resolution, width, height), 
  max_valid_th_(max_valid_th), min_valid_th_(min_valid_th)
{
    mask_images_.reset(new MaskImages(masks_msg_vec));
    depth_image_.reset(new DepthImage(depth_msg, camera_info_msg));
}

LSAMapGenerator::LSAMapGenerator(
    std::string frame_id, float resolution, int width, int height, 
    float max_valid_th, float min_valid_th, float noise_contour_th)
: Map(frame_id, resolution, width, height), 
  max_valid_th_(max_valid_th), min_valid_th_(min_valid_th)
{
    mask_images_.reset(new MaskImages(noise_contour_th));
    depth_image_.reset(new DepthImage());
}

void LSAMapGenerator::set_origin(float ox, float oy)
{
    ox_ = ox - max_valid_th_ / 2;
    oy_ = oy - max_valid_th_ / 2;
}

void LSAMapGenerator::update_image_infos(
    std::vector<sensor_msgs::msg::Image> & masks_msg_vec, 
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg)
{
    mask_images_->set_mask(masks_msg_vec);
    depth_image_->set_depth_image_from_msg(depth_msg);
    depth_image_->set_camera_model_from_msg(camera_info_msg);
}

bool LSAMapGenerator::create_grid_map_from_contours(const tf2::Transform & tf_camera_to_odom)
{
    // Reset Map
    reset_map();
	RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Started to create map");
    try{
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> occupied_pc_vec;
        contours_to_3d_point(occupied_pc_vec);

        RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Completed to create pointcloud");

        // Transform coordinate camera frame->odom
        for(auto &pc: occupied_pc_vec){
            pcl_ros::transformPointCloud(*pc, *pc, tf_camera_to_odom);
        }
        RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Transform Pointcloud");

        // Point xy -> Grid xy
        plot_occupied_and_raycast(occupied_pc_vec);
        
	    RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Completed to create map");
        return true;
    }catch(std::exception & e){
        RCLCPP_ERROR(rclcpp::get_logger("lang_sam_to_map"), "Exeption Error: %s", e.what());
        return false;
    }
    // double t = tf2::getYaw(lang_sam_map_.info.origin.orientation);
    // RCLCPP_INFO(get_logger(), "Map Origin: x: %lf, y: %lf, t: %lf", 
    //     lang_sam_map_.info.origin.position.x, lang_sam_map_.info.origin.position.y, t);
}

void LSAMapGenerator::contours_to_3d_point(
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & occupied_pc_vec)
{
    std::vector<std::vector<cv::Point>> contours;
    mask_images_->get_contours(contours);

    // contours points and depth -> pcl
	for(size_t i=0; i<contours.size(); ++i){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        pointcloud->points.reserve(contours[i].size());
        for(auto &p: contours[i]){
            int rows, cols;
            mask_images_->get_image_size(rows, cols);
            if(p.y > rows - 10) continue;
            cv::Point3d xyz;
            if(!depth_image_->uv_to_xyz(p.x, p.y, xyz)) continue;
            pcl::PointXYZ p_xyz(xyz.x, xyz.y, xyz.z);
            pointcloud->points.emplace_back(p_xyz);
            
		}
        occupied_pc_vec.emplace_back(pointcloud);
	}
}

void LSAMapGenerator::plot_occupied_and_raycast(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & occupied_pc_vec)
{
    pcl::PointCloud<pcl::PointXYZ>::iterator pt;
    for(auto &pc: occupied_pc_vec){
        for (pt=pc->points.begin(); pt < pc->points.end(); pt++){
            bresenham(
                static_cast<int>(((*pt).x - ox_) / resolution_), 
                static_cast<int>(((*pt).y - oy_) / resolution_));
            int ix, iy;
            if(!xy_to_index((*pt).x, (*pt).y, ix, iy)) continue;
            data_[ix][iy] = 100;
        }
    }
}

void LSAMapGenerator::bresenham(int x_e, int y_e)
{
    int x0 = static_cast<int>(width_ / 2);
    int y0 = static_cast<int>(height_ / 2);
    if(is_out_range(x0, y0)) return;

    int x1 = x_e;
    int y1 = y_e;

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

bool LSAMapGenerator::get_visualize_msg(
    sensor_msgs::msg::Image & output, 
    sensor_msgs::msg::Image::ConstSharedPtr base, 
    std::vector<sensor_msgs::msg::RegionOfInterest> & boxes)
{
    cv::Mat cv_vis;
    Image image;
    image.img_msg_to_cv(base, cv_vis);
    mask_images_->draw_mask_contours_bbox(cv_vis);
	for(auto &box: boxes){
		// バウンディングボックスを描画
		cv::rectangle(cv_vis, 
            cv::Point(box.x_offset, box.y_offset), 
            cv::Point(box.x_offset+box.width, box.y_offset+box.height), 
            cv::Scalar(0, 0, 255), 1);
	}
    return image.cv_to_msg(base->header, cv_vis, output);
}

LSAMapGenerator::~LSAMapGenerator(){}
    
} // namespace lang_sam_to_map
