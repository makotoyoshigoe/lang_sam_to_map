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

bool LSAMapGenerator::create_grid_map_from_contours(
    const tf2::Transform & tf_camera_to_base, 
    const tf2::Transform & tf_base_to_odom)
{
    // Reset Map
    reset_map();
	RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Started to create map");
    try{
        RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Number of Contours: %ld", occupied_pc_vec_.size());
        for(auto & c: occupied_pc_vec_){
            RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Number of Point: %ld", c->points.size());
        }
        // Convert Contours 2D to 3D Point
        contours_to_3d_point();
        RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Completed to create pointcloud");

        // Transform coordinate camera frame->odom
        // RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), 
        // "B: x, y, z: %f, %f, %f", 
        // occupied_pc_vec_[0]->points[0].x, occupied_pc_vec_[0]->points[0].y, occupied_pc_vec_[0]->points[0].z);
        // Transform coordinate camera frame->base
        for(auto & pc: occupied_pc_vec_){
            pcl_ros::transformPointCloud(*pc, *pc, tf_camera_to_base);
            // for(auto & p: pc->points){
            //     float r = hypot(p.x, p.y);
            //     // RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "R: %f", r);
            //     if(r > max_valid_th_){
            //         float theta = atan2(p.y, p.x);
            //         p.x = max_valid_th_ * cos(theta);
            //         p.y = max_valid_th_ * sin(theta);
            //     }
            // }
        }
        for(auto & pc: occupied_pc_vec_){
            pcl_ros::transformPointCloud(*pc, *pc, tf_base_to_odom);
        }
        // RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), 
        // "A: x, y, z: %f, %f, %f", 
        // occupied_pc_vec_[0]->points[0].x, occupied_pc_vec_[0]->points[0].y, occupied_pc_vec_[0]->points[0].z);
        RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Transform Pointcloud");

        // for(auto & pc: occupied_pc_vec_){
        //     for(auto & p: pc->points){
        //         RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "x, y, z: %f, %f, %f");
                // float r = hypot(p.x, p.y);
                // RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "R: %f", r);
                // if(r > max_valid_th_){
                //     float theta = atan2(p.y, p.x);
                //     p.x = max_valid_th_ * cos(theta) - ox_;
                //     p.y = max_valid_th_ * sin(theta) - oy_;
                // }
        //     }
        // }

        // Connect Occupied Grid
        connect_occupied_grid();
        RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Connect Occupied Grid");

        // Point xy -> Grid xy
        plot_occupied_and_raycast();
        
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

void LSAMapGenerator::contours_to_3d_point(void)
{
    occupied_pc_vec_.clear();

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
        occupied_pc_vec_.emplace_back(pointcloud);
	}
}



void LSAMapGenerator::connect_occupied_grid(void)
{
    occupied_grid_.clear();
    for(auto &pc: occupied_pc_vec_){
        for(size_t i=0; i<pc->points.size()-1; ++i){
            if(hypot(pc->points[i].x-pc->points[i+1].x, pc->points[i].y-pc->points[i+1].y) > 1.0) continue;
            bresenham(
                static_cast<int>((pc->points[i].x - ox_) / resolution_), 
                static_cast<int>((pc->points[i].y - oy_) / resolution_), 
                static_cast<int>((pc->points[i+1].x - ox_) / resolution_), 
                static_cast<int>((pc->points[i+1].y - oy_) / resolution_));
        }
    }
}

void LSAMapGenerator::bresenham(
    int x_s, int y_s, int x_e, int y_e)
{
    int x0 = x_s;
    int y0 = y_s;

    int x1 = x_e;
    int y1 = y_e;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    
    int err = dx - dy;

    while (true) {
        occupied_grid_.emplace_back(Grid{x0, y0});
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) {err -= dy; x0 += sx;}
        if (e2 <  dx) {err += dx; y0 += sy;}
    }
}

void LSAMapGenerator::plot_occupied_and_raycast(void)
{
    // for(auto & p: occupied_grid_){
    //     bresenham_fill(p.x, p.y);
    //     if(is_out_range(p.x, p.y)) continue;
    //     data_[p.x][p.y] = 100;
    // }
    pcl::PointCloud<pcl::PointXYZ>::iterator pt;
    for(auto &pc: occupied_pc_vec_){
        for (pt=pc->points.begin(); pt < pc->points.end(); pt++){
            bresenham_fill(
                static_cast<int>(((*pt).x - ox_) / resolution_), 
                static_cast<int>(((*pt).y - oy_) / resolution_));
            int ix, iy;
            if(!xy_to_index((*pt).x, (*pt).y, ix, iy)) continue;
            data_[ix][iy] = 100;
        }
    }
}

void LSAMapGenerator::bresenham_fill(int x_e, int y_e)
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
        if (e2 > -dy) {err -= dy; x0 += sx;}
        if (e2 <  dx) {err += dx; y0 += sy;}
    }
}

bool LSAMapGenerator::get_visualize_msg(
    bool raw, 
    sensor_msgs::msg::Image & output, 
    sensor_msgs::msg::Image::ConstSharedPtr base, 
    std::vector<sensor_msgs::msg::RegionOfInterest> & boxes)
{
    cv::Mat cv_vis;
    Image image;
    image.img_msg_to_cv(base, cv_vis);
    mask_images_->draw_mask_contours_bbox(cv_vis, raw);
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
