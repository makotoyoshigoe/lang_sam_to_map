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
    declare_parameter("road_scan.front_angle_abs", 0.523);
    declare_parameter("control_freq", 20);
    declare_parameter("base_frame_id", "base_footprint");
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("controller.linear_vel.max", 0.2);
    declare_parameter("controller.linear_vel.min", 0.0);
    declare_parameter("controller.angular_vel.max", 1.0);
    declare_parameter("controller.angular_vel.min", -0.5);
    declare_parameter("controller.linear_acc_th", 0.05);
    declare_parameter("controller.linear_dec_th", 0.05);
    declare_parameter("controller.angular_acc_th", 0.1);
    declare_parameter("controller.angular_dec_th", -0.1);
    declare_parameter("controller.kp", 1.);
    declare_parameter("controller.ki", 0.);
    declare_parameter("controller.kd", 0.);
    declare_parameter("potential.critical_distance", 3.0);
    declare_parameter("potential.repulsive_gain_map", 1.0);
    declare_parameter("potential.repulsive_gain_scan", 1.0);
    declare_parameter("potential.attractive_gain", 3.0);
    declare_parameter("potential.detect_angle_start", -45.0);
    declare_parameter("potential.detect_angle_end", 45.0);
    declare_parameter("potential.detect_angle_division_num", 5);
    declare_parameter("potential.repulsive_min_distance", 0.25);
}

void LsaNavController::init_param(void)
{
    float max_angle = get_parameter("road_scan.max_angle_abs").as_double();
    float min_angle = get_parameter("road_scan.min_angle_abs").as_double();
    float angle_increment = get_parameter("road_scan.angle_increment").as_double();
    float max_range = get_parameter("road_scan.max_range").as_double();
    float min_range = get_parameter("road_scan.min_range").as_double();
    float front_angle_abs = get_parameter("road_scan.front_angle_abs").as_double();
    control_freq_ = get_parameter("control_freq").as_int();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    float lin_max_vel = get_parameter("controller.linear_vel.max").as_double();
    float lin_min_vel = get_parameter("controller.linear_vel.min").as_double();
    float ang_max_vel = get_parameter("controller.angular_vel.max").as_double();
    float ang_min_vel = get_parameter("controller.angular_vel.min").as_double();
    float lin_acc_th = get_parameter("controller.linear_acc_th").as_double();
    float lin_dec_th = get_parameter("controller.linear_dec_th").as_double();
    float ang_acc_th = get_parameter("controller.angular_acc_th").as_double();
    float ang_dec_th = get_parameter("controller.angular_dec_th").as_double();
    float kp = get_parameter("controller.kp").as_double();
    float ki = get_parameter("controller.ki").as_double();
    float kd = get_parameter("controller.kd").as_double();
    // road_scan_creator_.reset(new RoadScanCreator(max_angle, min_angle, angle_increment, max_range, min_range, front_angle_abs));
    //controller_.reset(new Controller(
    //    lin_max_vel, lin_min_vel, ang_max_vel, ang_min_vel, 
    //    lin_acc_th, lin_dec_th, ang_acc_th, ang_dec_th, 
    //    kp, ki, kd, 1 / (float)control_freq_));
    float critical_distance = get_parameter("potential.critical_distance").as_double(); 
    float repulsive_gain_map = get_parameter("potential.repulsive_gain_map").as_double();
    float repulsive_gain_scan = get_parameter("potential.repulsive_gain_scan").as_double();
    float attractive_gain = get_parameter("potential.attractive_gain").as_double();
    float detect_angle_start = get_parameter("potential.detect_angle_start").as_double() * M_PI / 180.0;
    float detect_angle_end = get_parameter("potential.detect_angle_end").as_double() * M_PI / 180.0;
    int detect_angle_division_num = get_parameter("potential.detect_angle_division_num").as_int();
    float min_distance = get_parameter("potential.repulsive_min_distance").as_double();
    potential_controller_.reset(new PotentialController(
        critical_distance, 
        repulsive_gain_map, repulsive_gain_scan, attractive_gain, 
        detect_angle_start, detect_angle_end, detect_angle_division_num, 
        min_distance, lin_max_vel, ang_max_vel));
}

void LsaNavController::init_pubsub(void)
{
    sub_lsa_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "lang_sam_map", rclcpp::QoS(10), std::bind(&LsaNavController::cb_lsa_map, this, std::placeholders::_1));
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&LsaNavController::cb_scan, this, std::placeholders::_1));
    pub_road_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("scan/road", rclcpp::SensorDataQoS());
    pub_map_test_ = create_publisher<nav_msgs::msg::OccupancyGrid>("lsa_map/test", rclcpp::QoS(10));
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel/lsa_nav", rclcpp::QoS(10));
}

void LsaNavController::cb_lsa_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    potential_controller_->set_map_data(msg);
    receive_map_ = true;
}

void LsaNavController::cb_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    potential_controller_->set_scan_data(msg);
    scan_frame_id_ = msg->header.frame_id;
    receive_scan_ = true;
}

void LsaNavController::main_loop(void)
{
    if(!init_tf_) init_tf();
    //if(!receive_scan_ || !receive_map_) return;
    if(!receive_scan_) return;
    geometry_msgs::msg::Pose2D odom_to_base_pose, base_to_lidar_pose;
    if(!get_tf_pose(base_frame_id_, odom_frame_id_, odom_to_base_pose)) return;
    if(!get_tf_pose(scan_frame_id_, odom_frame_id_, base_to_lidar_pose)) return;
    CmdVel cmd_vel = potential_controller_->get_cmd_vel(odom_to_base_pose, base_to_lidar_pose);
    publish_cmd_vel(cmd_vel);
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

bool LsaNavController::get_tf_pose(
    const std::string & target_frame, 
    const std::string & source_frame,
    geometry_msgs::msg::Pose2D & pose2d)
{
    geometry_msgs::msg::PoseStamped ident;
	ident.header.frame_id = target_frame;
	ident.header.stamp = rclcpp::Time(0);
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

	geometry_msgs::msg::PoseStamped pose;
	try {
		this->tf_buffer_->transform(ident, pose, source_frame);
	} catch (tf2::TransformException & e) {
		RCLCPP_WARN(
		  get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
	pose2d.x = pose.pose.position.x;
	pose2d.y = pose.pose.position.y;
	pose2d.theta = tf2::getYaw(pose.pose.orientation);
	return true;
}

int LsaNavController::get_loop_freq(void){return control_freq_;}

void LsaNavController::publish_cmd_vel(CmdVel & cmd_vel)
{
    // Convert Force to CmdVel
    geometry_msgs::msg::Twist msg;
    msg.angular.z = cmd_vel.angular_vel;
    msg.linear.x = cmd_vel.linear_vel;
    pub_cmd_vel_->publish(msg);
}
    
} // namespace lsa_nav_controller

