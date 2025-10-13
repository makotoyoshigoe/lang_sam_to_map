// SPd.x-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPd.x-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2/convert.h>

#include "lang_sam_to_map/core/lsa_map_generator.hpp"


namespace lang_sam_to_map
{
LSAMapGenerator::LSAMapGenerator(
    std::string frame_id, float resolution, 
    float max_valid_th, float min_valid_th, 
    float noise_contour_area_th, float connect_grid_th, 
    float map_offset_x, float map_offset_y)
: Map(frame_id, resolution, max_valid_th/resolution, max_valid_th/resolution, map_offset_x, map_offset_y), 
  max_valid_th_(max_valid_th), min_valid_th_(min_valid_th), connect_grid_th_(connect_grid_th)
{
    mask_images_.reset(new MaskImages(noise_contour_area_th));
    depth_image_.reset(new DepthImage());
}

void LSAMapGenerator::set_origin(float ox, float oy, geometry_msgs::msg::Quaternion oq)
{
    ot_ = tf2::getYaw(oq);
    oq_ = oq;
    float dx = map_offset_x_ * cos(ot_) - map_offset_y_ * sin(ot_);
    float dy = map_offset_x_ * sin(ot_) + map_offset_y_ * cos(ot_);
    ox_ = ox - dx;
    oy_ = oy - dy;
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
    const tf2::Transform & tf_camera_to_base)
{
    // Reset Map
    reset_map();
	RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Started to create map");
    try{
        // Convert Contours 2D to 3D Point
        contours_to_3d_point();
        RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Completed to create pointcloud");

        // Transform coordinate camera frame->odom
        for(auto & pc: occupied_pc_vec_){
            pcl_ros::transformPointCloud(*pc, *pc, tf_camera_to_base);
            for(auto & p: pc->points){
                p.x += ox_ + map_offset_x_;
                p.y += oy_ + map_offset_y_;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Transform Pointcloud");

        // Connect Occupied Grid
        connect_occupied_grid();
        RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Connect Occupied Grid");

        // Point xy -> Grid xy
        plot_occupied_and_raycast(true);
	    RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Completed to create map");
        return true;
    }catch(std::exception & e){
        RCLCPP_ERROR(rclcpp::get_logger("lang_sam_to_map"), "Exeption Error: %s", e.what());
        return false;
    }
}

void LSAMapGenerator::contours_to_3d_point(void)
{
    occupied_pc_vec_.clear();

    std::vector<std::vector<cv::Point>> contours;
    mask_images_->get_contours(contours);

    // contours points and depth -> pcl
	for(size_t i=0; i<contours.size(); ++i){
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto &p: contours[i]){
            int rows, cols;
            mask_images_->get_image_size(rows, cols);
            if(p.y > rows - 10) continue;
            cv::Point3d xyz;
            if(!depth_image_->uv_to_xyz(p.x, p.y, xyz)) continue;
            pcl::PointXYZ p_xyz(xyz.x, xyz.y, std::min(xyz.z, (double)max_valid_th_));
            pointcloud->points.emplace_back(p_xyz);
		}
        occupied_pc_vec_.emplace_back(pointcloud);
	}
}

void LSAMapGenerator::connect_occupied_grid(void)
{
    occupied_grid_.clear();
    for(auto &pc: occupied_pc_vec_){
        if(pc->points.size() == 0) continue;
        for(size_t i=0; i<pc->points.size()-1; ++i){
            if(hypot(pc->points[i].x-pc->points[i+1].x, pc->points[i].y-pc->points[i+1].y) > connect_grid_th_) continue;
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
    Grid p0{x_s, y_s}, p1{x_e, y_e};
    Grid d{std::abs(p1.x - p0.x), std::abs(p1.y - p0.y)};
    Grid s{(p0.x < p1.x) ? 1 : -1, (p0.y < p1.y) ? 1 : -1};
    
    int err = d.x - d.y;

    while (true) {
        occupied_grid_.emplace_back(Grid{p0.x, p0.y});
        if (p0.x == p1.x && p0.y == p1.y) break;
        int e2 = 2 * err;
        if (e2 > -d.y) {err -= d.y; p0.x += s.x;}
        if (e2 <  d.x) {err += d.x; p0.y += s.y;}
    }
}

void LSAMapGenerator::plot_occupied_and_raycast(bool mode)
{
    if(mode){
        for(auto & p: occupied_grid_){
            bresenham_fill(p.x, p.y);
            if(is_out_range(p.x, p.y)) continue;
            data_[p.x][p.y] = 100;
        }
    }else{
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
}

void LSAMapGenerator::bresenham_fill(int x_e, int y_e)
{
    int x, y;
    xy_to_index(ox_ + map_offset_x_, oy_ + map_offset_y_, x, y);
    Grid p0{x, y}, p1{x_e, y_e};
    Grid d{std::abs(p1.x - p0.x), std::abs(p1.y - p0.y)};
    Grid s{(p0.x < p1.x) ? 1 : -1, (p0.y < p1.y) ? 1 : -1};
    if(is_out_range(p0.x, p0.y)) return;

    int err = d.x - d.y;

    while (true) {
        if(is_out_range(p0.x, p0.y)) break;
        if (p0.x == p1.x && p0.y == p1.y) break;

        if(data_[p0.x][p0.y] != 100) data_[p0.x][p0.y] = 0;

        int e2 = 2 * err;
        if (e2 > -d.y) {err -= d.y; p0.x += s.x;}
        if (e2 <  d.x) {err += d.x; p0.y += s.y;}
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
