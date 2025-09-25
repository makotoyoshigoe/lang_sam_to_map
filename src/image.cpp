// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include "lang_sam_to_map/image.hpp"

namespace lang_sam_to_map
{

Image::Image(){}

bool Image::img_msg_to_cv(
    sensor_msgs::msg::Image::ConstSharedPtr img_msg,
    cv::Mat& cv_image)
{
    try {
        cv_image = cv_bridge::toCvShare(img_msg)->image.clone();
        return true;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("lang_sam_to_map"), "cv_bridge exception: %s", e.what());
        return false;
    } catch (cv::Exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("lang_sam_to_map"), "OpenCV exception: %s", e.what());
        return false;
    }
}

bool Image::cv_to_msg(
    const std_msgs::msg::Header &header, 
    const cv::Mat& input_img, 
    sensor_msgs::msg::Image& msg)
{
    try {
        msg = *cv_bridge::CvImage(header, "rgb8", input_img).toImageMsg();
        return true;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("lang_sam_to_map"), "cv_bridge exception: %s", e.what());
        return false;
    } catch (cv::Exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("lang_sam_to_map"), "OpenCV exception: %s", e.what());
        return false;
    }
}

Image::~Image(){}
 
} // namespace lang_sam_to_map
