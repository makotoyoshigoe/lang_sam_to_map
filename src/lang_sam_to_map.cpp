// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/lang_sam_to_map.hpp"
#include "lang_sam_to_map/core/rgbd_pointcloud_converter.hpp"
#include "lang_sam_to_map/core/lsa_map_generator.hpp"

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
    vg_leaf_size_ = this->get_parameter("pointcloud.voxel_grid.leaf_size").as_double();
    min_valid_th_ = this->get_parameter("valid_threshold.min").as_double();
    max_valid_th_ = this->get_parameter("valid_threshold.max").as_double();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    map_resolution_ = this->get_parameter("map.resolution").as_double();
    rgbd_pc_converter_.reset(new RGBDPointcloudConverter(vg_leaf_size_, min_valid_th_, max_valid_th_));
}

void LangSamToMap::init_pubsub(void)
{
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
	//lang_sam_map_.header.frame_id = odom_frame_id_;
	//lang_sam_map_.info.resolution = map_resolution_;
	map_width_ = max_valid_th_ / map_resolution_;
	map_height_ = max_valid_th_ / map_resolution_;
	//lang_sam_map_.info.width = map_width_;
	//lang_sam_map_.info.height = map_height_;
	//lang_sam_map_.info.origin.position.z = 0.;
	//lang_sam_map_.data.resize(map_width_*map_height_);
}

void LangSamToMap::cb_message(
        sensor_msgs::msg::Image::ConstSharedPtr depth,
        sensor_msgs::msg::Image::ConstSharedPtr color,
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
{
    if(!init_tf_) init_tf();
    if(publish_pointcloud_ && color != nullptr && depth != nullptr){
        rgbd_pc_converter_->update_images_info(color, depth, camera_info);
        publish_pointcloud();
    }
    if(!processing_){
        color_ = color;
        depth_ = depth;
        camera_info_ = camera_info;
        init_msg_receive_ = true;
    }
}

void LangSamToMap::publish_pointcloud(void)
{
    // RGB画像、深度画像->3次元色付き点群
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>), down_sampled_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    rgbd_pc_converter_->create_point_cloud(pc);
    rgbd_pc_converter_->down_sampling(pc, down_sampled_pc);

    // Publish Pointcloud
    sensor_msgs::msg::PointCloud2 output_msg = rgbd_pc_converter_->pcl_to_msg(down_sampled_pc);
    output_msg.header.stamp = now();
    pub_color_pc2_->publish(output_msg);
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
            double ox, oy;
            if(!get_odom(ox, oy)) return;

            lsa_map_generator->set_origin(ox - max_valid_th_ / 2, oy - max_valid_th_ / 2);

            // Get TF camera frame->odom
            tf2::Transform tf_camera_to_odom;
            if(!get_pose_from_camera_to_odom(color_->header.frame_id, tf_camera_to_odom)) return;

            lsa_map_generator->create_grid_map_from_contours(tf_camera_to_odom);

            nav_msgs::msg::OccupancyGrid lang_sam_map = lsa_map_generator->get_map_msg();

			pub_lang_sam_map_->publish(lang_sam_map);

            // Create Visualize Masks, Contours, BBox and Publish it
            cv::Mat vis_mask = lsa_map_generator->get_visalize_image(color_, response_msg->boxes);
            publish_vis_mask(vis_mask);
        }
    }
    last_map_publish_t_ = this->get_clock()->now();
    processing_ = false;
}

void LangSamToMap::publish_vis_mask(cv::Mat & input_img)
{
    sensor_msgs::msg::Image msg;
    bool cvt_vis = cv_to_msg(input_img, msg);
    if(!cvt_vis) return;
    pub_vis_mask_->publish(msg);
}

bool LangSamToMap::cv_to_msg(
    cv::Mat & input_img, 
    sensor_msgs::msg::Image & msg)
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
