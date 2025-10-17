// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <tf2_ros/create_timer_ros.h>
#include <tf2/utils.hpp>
#include <tf2/convert.hpp>

#include "lsa_nav_controller/lsa_nav_controller.hpp"

namespace lsa_nav_controller
{
LsaNavController::LsaNavController() : Node("lsa_nav_controller")
{
    declare_param();
    init_param();
    init_pubsub();
}

LsaNavController::~LsaNavController(){}

void LsaNavController::declare_param(void)
{
    declare_parameter("road_scan.max_angle_abs", 1.48);
    declare_parameter("road_scan.min_angle_abs", 1.04);
    declare_parameter("road_scan.angle_increment", 0.00436);
    declare_parameter("road_scan.max_range", 5.);
    declare_parameter("road_scan.min_range", 0.1);
    declare_parameter("control_freq", 20);
    declare_parameter("base_frame_id", "base_footprint");
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("controller.linear_vel.max", 0.2);
    declare_parameter("controller.linear_vel.min", 0.0);
    declare_parameter("controller.angular_vel.max", 0.5);
    declare_parameter("controller.angular_vel.min", -0.5);
    declare_parameter("controller.linear_acc.max", 0.05);
    declare_parameter("controller.linear_acc.min", 0.0);
    declare_parameter("controller.angular_acc.max", 0.1);
    declare_parameter("controller.angular_acc.mim", -0.1);
}

void LsaNavController::init_param(void)
{
    float max_angle = get_parameter("road_scan.max_angle_abs").as_double();
    float min_angle = get_parameter("road_scan.min_angle_abs").as_double();
    float angle_increment = get_parameter("road_scan.angle_increment").as_double();
    float max_range = get_parameter("road_scan.max_range").as_double();
    float min_range = get_parameter("road_scan.min_range").as_double();
    control_freq_ = get_parameter("control_freq").as_int();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    road_scan_creator_.reset(new RoadScanCreator(max_angle, min_angle, angle_increment, max_range, min_range));
}

void LsaNavController::init_pubsub(void)
{
    sub_lsa_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "lang_sam_map", rclcpp::QoS(10), std::bind(&LsaNavController::cb_lsa_map, this, std::placeholders::_1));
    pub_road_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("scan/road", rclcpp::SensorDataQoS());
    pub_map_test_ = create_publisher<nav_msgs::msg::OccupancyGrid>("lsa_map/test", rclcpp::QoS(10));
}

void LsaNavController::cb_lsa_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    road_scan_creator_->update_map(msg);
}

void LsaNavController::main_loop(void)
{
    if(!init_tf_) init_tf();
    geometry_msgs::msg::Pose2D map_to_base_pose;
    if(!get_odom(map_to_base_pose)) return;
    road_scan_creator_->create_road_scan(map_to_base_pose);
    publish_road_scan();
    // nav_msgs::msg::OccupancyGrid map;
    // road_scan_creator_->get_map_msg(map);
    // pub_map_test_->publish(map);
}

void LsaNavController::init_tf(void)
{
    try{
        tf_buffer_.reset();
        tf_listener_.reset();
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(), get_node_timers_interface(),
            create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
        tf_buffer_->setCreateTimerInterface(timer_interface);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        init_tf_ = true;
        RCLCPP_INFO(get_logger(), "Initialized TF");
    }catch(std::exception & e){
        RCLCPP_ERROR(get_logger(), "Exeption Error: %s", e.what());
        init_tf_ = false;
    }
}

bool LsaNavController::get_odom(
    geometry_msgs::msg::Pose2D & odom)
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
	odom.x = odom_pose.pose.position.x;
	odom.y = odom_pose.pose.position.y;
	odom.theta = tf2::getYaw(odom_pose.pose.orientation);
    // RCLCPP_INFO(get_logger(), "x, y, theta: %f, %f, %f", odom.x, odom.y, 180*odom.theta/M_PI);
	return true;
}

void LsaNavController::publish_road_scan(void)
{
    sensor_msgs::msg::LaserScan scan_msg;
    road_scan_creator_->get_scan_msg(scan_msg);
    scan_msg.header.frame_id = base_frame_id_;
    scan_msg.header.stamp = now();
    pub_road_scan_->publish(scan_msg);
}

int LsaNavController::get_loop_freq(void){return control_freq_;}
    
} // namespace lsa_nav_controller

