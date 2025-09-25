// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/lang_sam_to_map.hpp"
#include "lang_sam_to_map/rgbd_pointcloud_converter.hpp"
#include "lang_sam_to_map/lsa_map_generator.hpp"

#include <cstdlib>
#include <ctime>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

#include <pcl_ros/transforms.hpp>

namespace lang_sam_to_map{
LangSamToMap::LangSamToMap(void)
 : Node("lang_sam_to_map"), 
   sync_(approximate_policy_(10), sub_depth_, sub_color_, sub_camera_info_), 
   processing_(false), init_tf_(false), last_map_publish_t_(now())
{
    declare_param();
    init_param();
    init_client();
    init_pubsub();
	init_map();
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

LangSamToMap::~LangSamToMap(){}

void LangSamToMap::declare_param(void)
{
    this->declare_parameter("lsa.text_prompt", "side walk");
    this->declare_parameter("lsa.box_threshold", 0.3);
    this->declare_parameter("lsa.text_threshold", 0.25);
    this->declare_parameter("node_freq", 20);
    this->declare_parameter("interval.time", 5.0);
    this->declare_parameter("interval.distance", 5);
    this->declare_parameter("pointcloud.publish", false);
    this->declare_parameter("pointcloud.voxel_grid.leaf_size", 0.025);
    this->declare_parameter("valid_threshold.min", 0.3);
    this->declare_parameter("valid_threshold.max", 10.0);
    this->declare_parameter("map.resolution", 0.1);
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("base_frame_id", "base_footprint");
}

void LangSamToMap::init_param(void)
{
    node_freq_ = this->get_parameter("node_freq").as_int();
    time_interval_ = this->get_parameter("interval.time").as_double();
    distance_interval_ = this->get_parameter("interval.distance").as_int();
    publish_pointcloud_ = this->get_parameter("pointcloud.publish").as_bool();
    min_valid_th_ = this->get_parameter("valid_threshold.min").as_double();
    max_valid_th_ = this->get_parameter("valid_threshold.max").as_double();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    map_resolution_ = this->get_parameter("map.resolution").as_double();
}

void LangSamToMap::init_pubsub(void)
{
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&LangSamToMap::cb_odom, this, std::placeholders::_1));
    pub_color_pc2_ = create_publisher<sensor_msgs::msg::PointCloud2>("color_cloud", rclcpp::QoS(10));
    pub_vis_mask_ = create_publisher<sensor_msgs::msg::Image>("visualized_mask", rclcpp::QoS(10));
	pub_lang_sam_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("lang_sam_map", rclcpp::QoS(10));
    sub_color_.subscribe(this, "/camera/camera/color/image_raw");
    sub_depth_.subscribe(this, "/camera/camera/aligned_depth_to_color/image_raw");
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
    RCLCPP_INFO(get_logger(), "Server was Launched");
}

void LangSamToMap::init_tf(void)
{
    tf_buffer_.reset();
    tf_listener_.reset();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface(),
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    init_tf_ = true;
}

void LangSamToMap::init_map(void)
{
	lang_sam_map_.header.frame_id = odom_frame_id_;
	lang_sam_map_.info.resolution = map_resolution_;
	map_width_ = max_valid_th_ / map_resolution_;
	map_height_ = max_valid_th_ / map_resolution_;
	lang_sam_map_.info.width = map_width_;
	lang_sam_map_.info.height = map_height_;
	lang_sam_map_.info.origin.position.z = 0.;
	lang_sam_map_.data.resize(map_width_*map_height_);
}

void LangSamToMap::cb_odom(
    nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    // double t = tf2::getYaw(msg->pose.pose.orientation);
    // lang_sam_map_.info.origin.position.x = msg->pose.pose.position.x - max_valid_th_ / 2;
    // lang_sam_map_.info.origin.position.y = msg->pose.pose.position.y - max_valid_th_ / 2;
    // lang_sam_map_.info.origin.orientation = msg->pose.pose.orientation;
    // lang_sam_map_.info.origin.orientation.x = -msg->pose.pose.orientation.x;
    // lang_sam_map_.info.origin.orientation.y = -msg->pose.pose.orientation.y;
    // lang_sam_map_.info.origin.orientation.z = -msg->pose.pose.orientation.z;
    // lang_sam_map_.info.origin.orientation.w = -msg->pose.pose.orientation.w;
    // lang_sam_map_.info.origin.position.x = 0. - min_valid_th_;
    // lang_sam_map_.info.origin.position.y = -max_valid_th_ / 2;
}

void LangSamToMap::cb_message(
        sensor_msgs::msg::Image::ConstSharedPtr depth,
        sensor_msgs::msg::Image::ConstSharedPtr color,
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
{
    if(!init_tf_) init_tf();
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

    // RGB画像、深度画像->3次元色付き点群
    auto rgbd_pc_cvt = std::make_unique<RGBDPointcloudConverter>(color_msg, depth_msg, camera_info);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>), down_sampled_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    rgbd_pc_cvt->create_point_cloud(pc, max_valid_th_, min_valid_th_);
    rgbd_pc_cvt->down_sampling(pc, down_sampled_pc, vg_leaf_size_);

    // Publish Pointcloud
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*down_sampled_pc, output_msg);
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

bool LangSamToMap::uv_to_xyz(
    image_geometry::PinholeCameraModel& cam_model,
	cv::Mat& cv_depth, 
    int u, int v, cv::Point3d& xyz)
{
    float z = static_cast<float>(cv_depth.at<uint16_t>(v, u)) * 0.001f;
    if(std::isnan(z) || z < min_valid_th_ || z > max_valid_th_){
        return false;
	}
    xyz = cam_model.projectPixelTo3dRay(cv::Point2d(u, v)) * z;
    return true;
}

bool LangSamToMap::get_pose_from_camera_to_base(
    std::string camera_frame_id,
    tf2::Transform& tf)
{
    rclcpp::Time time = rclcpp::Time(0);
    geometry_msgs::msg::TransformStamped tf_tmp;

    try {
        tf_tmp = tf_buffer_->lookupTransform(
            base_frame_id_, camera_frame_id, time, rclcpp::Duration::from_seconds(4.0));
        tf2::fromMsg(tf_tmp.transform, tf);
    } catch (tf2::TransformException& e) {
        RCLCPP_WARN(
            this->get_logger(), "Failed to compute camera pose(%s)", e.what());
        return false;
    }
    return true;
}

bool LangSamToMap::get_pose_from_camera_to_odom(
    std::string camera_frame_id,
    tf2::Transform& tf)
{
    rclcpp::Time time = rclcpp::Time(0);
    geometry_msgs::msg::TransformStamped tf_tmp;

    try {
        tf_tmp = tf_buffer_->lookupTransform(
            odom_frame_id_, camera_frame_id, time, rclcpp::Duration::from_seconds(4.0));
        tf2::fromMsg(tf_tmp.transform, tf);
    } catch (tf2::TransformException& e) {
        RCLCPP_WARN(
            this->get_logger(), "Failed to compute camera pose(%s)", e.what());
        return false;
    }
    return true;
}

bool LangSamToMap::send_request(void)
{
    processing_ = true;
    bool get_odom_flg = get_odom(lang_sam_map_.info.origin.position.x, lang_sam_map_.info.origin.position.y);
    request_msg_->image = *color_;
    if(init_request_) RCLCPP_INFO(get_logger(), "Send Request to Server, %lf second since last request", get_diff_time());
    else RCLCPP_INFO(get_logger(), "Send request for the first time");
    init_request_ = true;
    auto future = lsa_client_->async_send_request(
        request_msg_, 
        std::bind(&LangSamToMap::handle_process, this, std::placeholders::_1));
    return true;
}

bool LangSamToMap::get_odom(double &x, double &y)
{
    geometry_msgs::msg::PoseStamped ident;
	ident.header.frame_id = base_frame_id_;
	ident.header.stamp = rclcpp::Time(0);
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

	geometry_msgs::msg::PoseStamped odom_pose;
	try {
		this->tf_buffer_->transform(ident, odom_pose, odom_frame_id_);
	} catch (tf2::TransformException & e) {
		RCLCPP_WARN(
		  get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
	x = odom_pose.pose.position.x;
	y = odom_pose.pose.position.y;
	// t = tf2::getYaw(odom_pose.pose.orientation);

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
            auto lsa_map_generator = std::make_unique<LSAMapGenerator>(response_msg->masks, depth_, camera_info_, odom_frame_id_, map_resolution_, map_width_, map_height_);
            lang_sam_map_ = lsa_map_generator->get_map_msg(now());
            double ox, oy;
            if(!get_odom(ox, oy)) return;

            lsa_map_generator->set_origin(ox - max_valid_th_ / 2, oy - max_valid_th_ / 2);

            // Get TF camera frame->odom
            tf2::Transform tf_camera_to_odom;
            if(!get_pose_from_camera_to_odom(color_->header.frame_id, tf_camera_to_odom)) return;

            lsa_map_generator->create_grid_map_from_contours(tf_camera_to_odom);

            nav_msgs::msg::OccupancyGrid lang_sam_map = lsa_map_generator->get_map_msg(now());
            // ROS Message Vector to CV Vector
            // std::vector<cv::Mat> cv_bin_masks = msg_mask_to_binary(response_msg->masks);

            // Addweight binary
            // std::vector<cv::Mat> add_weighted_bin(1, add_weight_bin(cv_bin_masks));

            // Binary Mask to RGB Mask
            // std::vector<cv::Mat> cv_rgb_masks = bin_mask_to_rgb(add_weighted_bin);

            // Find Contours
            // std::vector<std::vector<std::vector<cv::Point>>> contours = find_each_mask_contours(cv_rgb_masks);

            // Connect neigboor neighborhood contours point


			// Transform contours 2D points to occupied grid map
			// bool create_map = create_grid_map_from_contours(contours, add_weighted_bin[0]);
			// if(create_map) pub_lang_sam_map_->publish(lang_sam_map_);
			pub_lang_sam_map_->publish(lang_sam_map);

            // Create Visualize Masks, Contours, BBox and Publish it
            // cv::Mat vis_mask = visualize_mask_contours_bbox(cv_rgb_masks, contours, response_msg->boxes);
            // publish_vis_mask(vis_mask);
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
    std::shared_ptr<sensor_msgs::msg::Image> mask_ptr;
    for(size_t i=0; i<masks_s; ++i){
        mask_ptr.reset(new sensor_msgs::msg::Image(masks[i]));
        img_msg_to_cv(mask_ptr, cv_vec[i]);
    }
    return cv_vec;
}

cv::Mat LangSamToMap::add_weight_bin(const std::vector<cv::Mat>& bin_masks)
{

    cv::Mat add_weighted_bin;
    add_weighted_bin = cv::Mat::zeros(bin_masks[0].rows, bin_masks[0].cols, bin_masks[0].type());
    for(auto &bin: bin_masks){
        bin.forEach<u_char>([&](uchar &pixel, const int position[]) -> void{
            if (pixel > 0) {
                add_weighted_bin.at<uint8_t>(position[0], position[1]) = 1;
            }
        });
    }
    return add_weighted_bin;
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
        cv::findContours(binary, contours[i], hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    }
    return contours;
}

bool LangSamToMap::create_grid_map_from_contours(
	std::vector<std::vector<std::vector<cv::Point>>>& contours, 
    cv::Mat bin_mask)
{
    // Reset Map
    std::fill(lang_sam_map_.data.begin(), lang_sam_map_.data.end(), -1);

    // Create Camera Model
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(camera_info_);

    // ROS->CV(depth)
    cv::Mat cv_depth;
    bool cvt_depth = img_msg_to_cv(depth_, cv_depth);
	if(!cvt_depth) return false;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> occupied_pc_vec;
	RCLCPP_INFO(get_logger(), "Started to create map");

	// contours points and depth -> pcl
	for(size_t i=0; i<contours.size(); ++i){
		for(size_t j=0; j<contours[i].size(); ++j){
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
            pointcloud->points.reserve(contours[i][j].size());
			for(auto &p: contours[i][j]){
				cv::Point3d xyz;
                bool cvt_res = uv_to_xyz(cam_model, cv_depth, p.x, p.y, xyz);
			    if(!cvt_res) continue;
                pcl::PointXYZ p_xyz(xyz.x, xyz.y, xyz.z);
                pointcloud->points.emplace_back(p_xyz);
			}
            occupied_pc_vec.emplace_back(pointcloud);
		}
	}
    pcl::PointCloud<pcl::PointXYZ>::Ptr unoccupied_pc(new pcl::PointCloud<pcl::PointXYZ>);
    unoccupied_pc->points.reserve(bin_mask.rows*bin_mask.cols);
    bin_mask.forEach<uchar>([&](uchar &pixel, const int position[]) -> void {
        if (pixel > 0) {
            cv::Point3d xyz;
            bool cvt_res = uv_to_xyz(cam_model, cv_depth, position[1], position[0], xyz);
            if(cvt_res){
                pcl::PointXYZ p_xyz(xyz.x, xyz.y, xyz.z);
                unoccupied_pc->points.emplace_back(p_xyz);
            }
        }
    });
	RCLCPP_INFO(get_logger(), "Completed to create pointcloud");
	
	// Get TF camera frame->odom
    tf2::Transform tf;
    bool get_tf = get_pose_from_camera_to_odom(color_->header.frame_id, tf);
    if(!get_tf) return false;

    // Transform coordinate camera frame->odom
    for(auto &pc: occupied_pc_vec){
        pcl_ros::transformPointCloud(*pc, *pc, tf);
    }
    // pcl_ros::transformPointCloud(*unoccupied_pc, *unoccupied_pc, tf);

    // Point xy -> Grid xy
	// z->x, x->-y
	pcl::PointCloud<pcl::PointXYZ>::iterator pt;
    // for (pt=unoccupied_pc->points.begin(); pt < unoccupied_pc->points.end()-1; pt++){
    //     int index = xy_to_index((*pt).x, (*pt).y);
    //     if(index == -1) continue;
    //     lang_sam_map_.data[index] = 0;
    // }
	for(auto &pc: occupied_pc_vec){
 		for (pt=pc->points.begin(); pt < pc->points.end(); pt++){
            bresenham(
                static_cast<int>(((*pt).x - lang_sam_map_.info.origin.position.x) / map_resolution_), 
                static_cast<int>(((*pt).y - lang_sam_map_.info.origin.position.y) / map_resolution_));
			int index = xy_to_index((*pt).x, (*pt).y);
            if(index != -1) lang_sam_map_.data[index] = 100;
		}
	}
    
	RCLCPP_INFO(get_logger(), "Completed to create map");
    double t = tf2::getYaw(lang_sam_map_.info.origin.orientation);
    // RCLCPP_INFO(get_logger(), "Map Origin: x: %lf, y: %lf, t: %lf", 
    //     lang_sam_map_.info.origin.position.x, lang_sam_map_.info.origin.position.y, t);
	return true;
}

int LangSamToMap::xy_to_index(double x, double y)
{
    int mx = static_cast<int>((x-lang_sam_map_.info.origin.position.x)/map_resolution_);
    int my = static_cast<int>((y-lang_sam_map_.info.origin.position.y)/map_resolution_);
    if(mx < 0 || mx >= map_width_ || my < 0 || my >= map_height_) return -1;
    return my * map_width_ + mx;
}

void LangSamToMap::bresenham(int x_e, int y_e)
{
    int x0 = static_cast<int>(map_width_ / 2);
    int y0 = static_cast<int>(map_height_ / 2);
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
        int fill_i = x0+y0*map_height_;
        if(fill_i < 0 || x0 > map_width_ || y0 > map_height_) break;
        if(fill_i != 0 || lang_sam_map_.data[fill_i] != 100) lang_sam_map_.data[fill_i] = 0;
        // RCLCPP_INFO(get_logger(), "fill index: %d", fill_i);

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

cv::Mat LangSamToMap::visualize_mask_contours_bbox(
        const std::vector<cv::Mat>& cv_rgb_masks, 
        const std::vector<std::vector<std::vector<cv::Point>>>& contours,
		const std::vector<sensor_msgs::msg::RegionOfInterest>& boxes)
{
    cv::Mat cv_color;
    img_msg_to_cv(color_, cv_color);
    size_t mask_s = cv_rgb_masks.size();
    for(size_t i=0; i<mask_s; ++i){
        cv::addWeighted(cv_color, 1.0, cv_rgb_masks[i], 0.5, 0.0, cv_color);
        cv::drawContours(cv_color, contours[i], -1, cv::Scalar(255, 0, 0), 1);
    }
	for(auto &box: boxes){
		// バウンディングボックスを描画
		cv::rectangle(cv_color, 
            cv::Point(box.x_offset, box.y_offset), 
            cv::Point(box.x_offset+box.width, box.y_offset+box.height), 
            cv::Scalar(0, 0, 255), 1);
	}
    return cv_color;
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
