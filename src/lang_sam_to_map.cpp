// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/lang_sam_to_map.hpp"

#include<cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <ctime>

namespace lang_sam_to_map{
LangSamToMap::LangSamToMap(void)
 : Node("lang_sam_to_map"), 
   sync_(approximate_policy_(10), sub_color_, sub_depth_, sub_camera_info_), 
   processing_(false), last_map_publish_t_(now())
{
    declare_param();
    init_param();
    init_vg_filter();
    init_client();
    init_pubsub();
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

LangSamToMap::~LangSamToMap(){}

void LangSamToMap::declare_param(void)
{
    this->declare_parameter("lsa.text_prompt", "side_walk");
    this->declare_parameter("lsa.box_threshold", 0.4);
    this->declare_parameter("lsa.text_threshold", 0.5);
    this->declare_parameter("node_freq", 20);
    this->declare_parameter("interval.time", 5.0);
    this->declare_parameter("interval.distance", 5);
    this->declare_parameter("pointcloud.publish", false);
    this->declare_parameter("pointcloud.voxel_grid.leaf_size", 0.025);
    this->declare_parameter("valid_threshold.min", 0.3);
    this->declare_parameter("valid_threshold.max", 10.0);
    this->declare_parameter("map.resolution", 0.1);
}

void LangSamToMap::init_param(void)
{
    node_freq_ = this->get_parameter("node_freq").as_int();
    time_interval_ = this->get_parameter("interval.time").as_double();
    distance_interval_ = this->get_parameter("interval.distance").as_int();
    publish_pointcloud_ = this->get_parameter("pointcloud.publish").as_bool();
    min_valid_th_ = this->get_parameter("valid_threshold.min").as_double();
    max_valid_th_ = this->get_parameter("valid_threshold.max").as_double();
    map_resolution_ = this->get_parameter("map.resolution").as_double();
}

void LangSamToMap::init_vg_filter(void)
{
    double leaf_size = this->get_parameter("pointcloud.voxel_grid.leaf_size").as_double();
    voxel_grid_filter_.reset(new pcl::VoxelGrid<pcl::PointXYZRGB>());
    voxel_grid_filter_->setLeafSize(leaf_size, leaf_size, leaf_size);
}

void LangSamToMap::init_pubsub(void)
{
    pub_color_pc2_ = create_publisher<sensor_msgs::msg::PointCloud2>("color_cloud", rclcpp::QoS(10));
    pub_vis_mask_ = create_publisher<sensor_msgs::msg::Image>("visualized_mask", rclcpp::QoS(10));
    sub_depth_.subscribe(this, "/camera/camera/color/image_raw");
    sub_color_.subscribe(this, "/camera/camera/aligned_depth_to_color/image_raw");
    sub_camera_info_.subscribe(this, "/camera/camera/color/camera_info");
    sync_.registerCallback(&LangSamToMap::cb_message, this);
}

void LangSamToMap::init_client(void)
{
    lsa_client_ = this->create_client<ros2_lang_sam_msgs::srv::TextSegmentation>(
        "/lang_sam_server/text_segment");
    request_msg_ = std::make_shared<ros2_lang_sam_msgs::srv::TextSegmentation::Request>();
    request_msg_->text_prompt = this->get_parameter("lsa.text_prompt").as_string();
    request_msg_->box_threshold = this->get_parameter("lsa.box_threshold").as_double();
    request_msg_->text_threshold = this->get_parameter("lsa.text_threshold").as_double();
    using namespace std::chrono_literals;
    while (!lsa_client_->wait_for_service(5s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    // rclcpp::Rate loop_rate(0.2);
    // loop_rate.sleep();
    RCLCPP_INFO(get_logger(), "Server was Launched");
}

void LangSamToMap::cb_message(
        sensor_msgs::msg::Image::ConstSharedPtr depth,
        sensor_msgs::msg::Image::ConstSharedPtr color,
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
{
    if(publish_pointcloud_){
        publish_pointcloud(color, depth, camera_info);
    }
    if(!processing_){
        color_ = color;
        depth_ = depth;
        camera_info_ = camera_info;
        init_msg_receive_ = true;
    }
}

void LangSamToMap::publish_pointcloud(
        sensor_msgs::msg::Image::ConstSharedPtr color_msg, 
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
{
    if(color_msg == nullptr || depth_msg == nullptr) return;

    // ROS->CV
    cv::Mat cv_color, cv_depth;
    bool cvt_color = img_msg_to_cv(color_msg, cv_color);
    bool cvt_depth = img_msg_to_cv(depth_msg, cv_depth);
    if(!cvt_color || !cvt_depth) return;
    cv::cvtColor(cv_color, cv_color, CV_BGR2RGB);

    // color, depth -> pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    create_pointcloud(cv_color, cv_depth, camera_info, pointcloud);

    // Down Sampling
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid_filter_->setInputCloud(pointcloud);
    voxel_grid_filter_->filter(*sampled_cloud);

    // Publish Pointcloud
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*sampled_cloud, output_msg);
    output_msg.header.frame_id = color_msg->header.frame_id;
    output_msg.header.stamp = now();
    pub_color_pc2_->publish(output_msg);
}

bool LangSamToMap::img_msg_to_cv(
    sensor_msgs::msg::Image::ConstSharedPtr img_msg,
    cv::Mat& cv_image)
{
    try {
        cv_image = cv_bridge::toCvShare(img_msg)->image.clone();
        return true;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
        return false;
    } catch (cv::Exception& e) {
        RCLCPP_WARN(get_logger(), "OpenCV exception: %s", e.what());
        return false;
    }
}

void LangSamToMap::create_pointcloud(
        cv::Mat& cv_color,
        cv::Mat& cv_depth, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info, 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud)
{
    // Reserve PointCloud Vector
    size_t data_s = cv_color.rows * cv_color.cols;
    pointcloud->points.reserve(data_s);

    // Create Camera Model
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera_info);

    // Create PointCloud
    int rows = cv_color.rows, cols = cv_color.cols;
    for (int v = 0; v < rows; ++v){
        for (int u = 0; u < cols; ++u) {
            float z = static_cast<float>(cv_depth.at<uint16_t>(v, u)) * 0.001f;
            if(std::isnan(z) || z < min_valid_th_ || z > max_valid_th_) continue;
            cv::Point3d xyz = uvz_to_xyz(cam_model, u, v, z);
            cv::Vec3b color = cv_color.at<cv::Vec3b>(v, u);
            pcl::PointXYZRGB p(
                xyz.x, xyz.y, xyz.z, color[2], color[1], color[0]);
            pointcloud->points.emplace_back(p);
        }
    }

    // Set PointCloud Infomation
    pointcloud->width = pointcloud->points.size();
	pointcloud->height = 1;
	pointcloud->is_dense = false;
}

cv::Point3d LangSamToMap::uvz_to_xyz(
    image_geometry::PinholeCameraModel& cam_model,
    int u, int v, float z)
{
    cv::Point3d xyz = cam_model.projectPixelTo3dRay(cv::Point2d(u, v)) * z;
    return xyz;
}

bool LangSamToMap::send_request(void)
{
    processing_ = true;
    request_msg_->image = *color_;
    if(init_request_) RCLCPP_INFO(get_logger(), "Send Request to Server, %lf second since last request", get_diff_time());
    else RCLCPP_INFO(get_logger(), "Send request for the first time");
    init_request_ = true;
    auto future = lsa_client_->async_send_request(
        request_msg_, 
        std::bind(&LangSamToMap::handle_process, this, std::placeholders::_1));
    return true;
}

void LangSamToMap::handle_process(
        rclcpp::Client<ros2_lang_sam_msgs::srv::TextSegmentation>::SharedFuture future)
{
    std::shared_ptr<ros2_lang_sam_msgs::srv::TextSegmentation::Response> response_msg;
    response_msg = future.get();
    if(response_msg == nullptr || !future.valid()){
        RCLCPP_INFO(get_logger(), "Response Message is Null or Failed Server Process");
    }else{
        RCLCPP_INFO(get_logger(), "Recieve Responce, mask size: %ld", response_msg->masks.size());
        if(response_msg->masks.size() != 0){
            // ROS Message Vector to CV Vector
            std::vector<cv::Mat> cv_bin_masks = msg_mask_to_binary(response_msg->masks);

            // Binary Mask to RGB Mask
            std::vector<cv::Mat> cv_rgb_masks = bin_mask_to_rgb(cv_bin_masks);

            // Find Contours
            std::vector<std::vector<std::vector<cv::Point>>> contours = find_each_mask_contours(cv_rgb_masks);

            // Create Map and Publish it
            RCLCPP_INFO(get_logger(), "0F: %ld, 1F: %ld, 2F: %ld", contours.size(), contours[0].size(), contours[0][0].size());

            // Create Visualize Masks and Publish it
            cv::Mat vis_mask = visualize_mask_contours(cv_rgb_masks, contours);
            publish_vis_mask(vis_mask);
        }
    }
    last_map_publish_t_ = this->get_clock()->now();
    processing_ = false;
}

std::vector<cv::Mat> LangSamToMap::msg_mask_to_binary(
    const std::vector<sensor_msgs::msg::Image>& masks)
{
    size_t masks_s = masks.size();
    std::vector<cv::Mat> cv_vec(masks_s);
    for(size_t i=0; i<masks_s; ++i){
        std::shared_ptr<sensor_msgs::msg::Image> mask_ptr;
        mask_ptr.reset(new sensor_msgs::msg::Image(masks[i]));
        img_msg_to_cv(mask_ptr, cv_vec[i]);
    }
    return cv_vec;
}

std::vector<cv::Mat> LangSamToMap::bin_mask_to_rgb(
    const std::vector<cv::Mat>& bin_masks)
{
    size_t masks_s = bin_masks.size();
    std::vector<cv::Mat> rgb_masks(masks_s);
    cv::Vec3b color(255, 255, 255);
    for(size_t i=0; i<masks_s; ++i){
        rgb_masks[i] = cv::Mat::zeros(bin_masks[i].rows, bin_masks[i].cols, CV_8UC3);
        bin_masks[i].forEach<uchar>([&](uchar &pixel, const int position[]) -> void {
            if (pixel > 0) {
                rgb_masks[i].at<cv::Vec3b>(position[0], position[1]) = color;
            }
        });
    }
    return rgb_masks;
}

cv::Mat LangSamToMap::visualize_mask_contours(
        const std::vector<cv::Mat>& cv_rgb_masks, 
        const std::vector<std::vector<std::vector<cv::Point>>>& contours)
{
    cv::Mat cv_color;
    img_msg_to_cv(color_, cv_color);
    size_t mask_s = cv_rgb_masks.size();
    for(size_t i=0; i<mask_s; ++i){
        cv::addWeighted(cv_color, 1.0, cv_rgb_masks[i], 0.5, 0.0, cv_color);
        cv::drawContours(cv_color, contours[i], -1, cv::Scalar(255, 0, 0), 1);
    }
    return cv_color;
}

std::vector<std::vector<std::vector<cv::Point>>> LangSamToMap::find_each_mask_contours(
    const std::vector<cv::Mat>& cv_rgb_masks)
{
    size_t mask_s = cv_rgb_masks.size();
    std::vector<std::vector<std::vector<cv::Point>>> contours(mask_s);
    std::vector<cv::Vec4i> hierarchy;
    for(size_t i=0; i<mask_s; ++i){
        cv::Mat gray, binary;
        cv::cvtColor(cv_rgb_masks[i], gray, CV_RGB2GRAY);
        cv::threshold(gray, binary, 150, 255, cv::THRESH_BINARY);
        cv::findContours(binary, contours[i], hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    }
    return contours;
}

void LangSamToMap::publish_vis_mask(cv::Mat& input_img)
{
    sensor_msgs::msg::Image msg;
    bool cvt_vis = cv_to_msg(input_img, msg);
    if(!cvt_vis) return;
    pub_vis_mask_->publish(msg);
}

bool LangSamToMap::cv_to_msg(
    cv::Mat& input_img, 
    sensor_msgs::msg::Image& msg)
{
    try {
        msg = *cv_bridge::CvImage(color_->header, "rgb8", input_img).toImageMsg();
        msg.header.stamp = now();
        return true;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_WARN(get_logger(), "cv_bridge exception: %s", e.what());
        return false;
    } catch (cv::Exception& e) {
        RCLCPP_WARN(get_logger(), "OpenCV exception: %s", e.what());
        return false;
    }
}

int LangSamToMap::get_node_freq(void){return node_freq_;}

bool LangSamToMap::flg_send_request(void)
{
    return (get_diff_time() >= time_interval_ || !init_request_) && init_msg_receive_ && !processing_;
}

double LangSamToMap::get_diff_time(void)
{
    return (this->get_clock()->now() - last_map_publish_t_).seconds();
}

}
