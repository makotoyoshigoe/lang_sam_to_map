// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include"lang_sam_to_map/camera.hpp"
#include<rclcpp/rclcpp.hpp>
#include<cv_bridge/cv_bridge.h>

namespace lang_sam_to_map{

bool img_msg_to_cv(
    sensor_msgs::msg::Image::ConstSharedPtr img_msg,
    cv::Mat& rgb_image)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try {
        cv_img_ptr = cv_bridge::toCvShare(img_msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("lc_fusion"), "cv_bridge exception: %s", e.what());
        return false;
    }

    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_bridge::toCvShare(img_msg)->image;

    cv::cvtColor(cv_image, rgb_image, CV_BGR2RGB);
    return true;
}

}

