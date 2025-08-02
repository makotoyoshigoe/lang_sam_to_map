// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include<string>

#include<rclcpp/rclcpp.hpp>
#include<sensor_msgs/msg/image.hpp>
#include<sensor_msgs/msg/camera_info.hpp>
#include<sensor_msgs/msg/point_cloud2.hpp>
#include<nav_msgs/msg/occupancy_grid.hpp>
#include<ros2_lang_sam_msgs/srv/text_segmentation.hpp>
#include<message_filters/subscriber.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<message_filters/time_synchronizer.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include<cv_bridge/cv_bridge.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl_ros/transforms.hpp>
#include <pcl/filters/voxel_grid.h>

namespace lang_sam_to_map{
class LangSamToMap : public rclcpp::Node{
public:
	LangSamToMap(void);
	~LangSamToMap();
    void declare_param(void);
    void init_param(void);
    void init_pubsub(void);
    void init_client(void);
    void init_tf(void);
	void init_map(void);
    void cb_message(
        sensor_msgs::msg::Image::ConstSharedPtr depth,
        sensor_msgs::msg::Image::ConstSharedPtr color,
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);
    void publish_pointcloud(
        sensor_msgs::msg::Image::ConstSharedPtr color_msg, 
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);
    bool img_msg_to_cv(
        sensor_msgs::msg::Image::ConstSharedPtr img_msg,
        cv::Mat& rgb_image);
    void create_pointcloud(
        cv::Mat& cv_color,
        cv::Mat& cv_depth, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info, 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud);
    cv::Point3d uv_to_xyz(
        image_geometry::PinholeCameraModel& cam_model,
		cv::Mat& cv_depth, 
        int u, int v);
    bool get_pose_from_camera_to_base(
        std::string camera_frame_id,
        tf2::Transform& tf);
    void init_vg_filter(void);
    bool send_request(void);
    void handle_process(
        rclcpp::Client<ros2_lang_sam_msgs::srv::TextSegmentation>::SharedFuture future);
    std::vector<cv::Mat> msg_mask_to_binary(
        const std::vector<sensor_msgs::msg::Image>& masks);
    std::vector<cv::Mat> bin_mask_to_rgb(
        const std::vector<cv::Mat>& bin_masks);
    std::vector<std::vector<std::vector<cv::Point>>> find_each_mask_contours(
        const std::vector<cv::Mat>& cv_rgb_masks);
	void create_grid_map_from_contours(
			std::vector<std::vector<std::vector<cv::Point>>>& contours);
	int xy_to_index(double x, double y);
    cv::Mat visualize_mask_contours(
        const std::vector<cv::Mat>& cv_rgb_masks, 
        const std::vector<std::vector<std::vector<cv::Point>>>& contours);
    void publish_vis_mask(
        cv::Mat& input_img);
    bool cv_to_msg(
        cv::Mat& input_img, 
        sensor_msgs::msg::Image& msg);
    int get_node_freq(void);
    bool flg_send_request(void);
    double get_diff_time(void);

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_color_pc2_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_vis_mask_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_color_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_;
    using approximate_policy_ = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo>;
    message_filters::Synchronizer<approximate_policy_> sync_;
    rclcpp::Client<ros2_lang_sam_msgs::srv::TextSegmentation>::SharedPtr lsa_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<ros2_lang_sam_msgs::srv::TextSegmentation::Request> request_msg_;
    std::shared_ptr<ros2_lang_sam_msgs::srv::TextSegmentation::Response> response_msg_;
    float box_th_, text_th_, min_valid_th_, max_valid_th_, time_interval_, map_resolution_;
    std::string text_prompt_;
    bool processing_, publish_pointcloud_, init_msg_receive_, init_request_, init_tf_;
    int node_freq_, distance_interval_, map_width_, map_height_;
    pcl::VoxelGrid<pcl::PointXYZRGB>::Ptr voxel_grid_filter_;
    rclcpp::Time last_map_publish_t_;
    std::string base_frame_id_, odom_frame_id_;

    sensor_msgs::msg::Image::ConstSharedPtr depth_;
    sensor_msgs::msg::Image::ConstSharedPtr color_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
	nav_msgs::msg::OccupancyGrid lang_sam_map_;
};
}
