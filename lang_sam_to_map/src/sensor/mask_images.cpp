// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include "lang_sam_to_map/sensor/mask_images.hpp"

namespace lang_sam_to_map{
MaskImages::MaskImages(const std::vector<sensor_msgs::msg::Image> masks_msg_vec) : Image()
{
    pre_process(masks_msg_vec);
}

MaskImages::MaskImages(float noise_contour_area_th)
: noise_contour_area_th_(noise_contour_area_th)
{}

void MaskImages::pre_process(const std::vector<sensor_msgs::msg::Image> masks_msg_vec)
{
    msg_mask_to_binary(masks_msg_vec);
    add_weight_bin();
    bin_mask_to_rgb();
    generate_contours_point();
}

void MaskImages::set_mask(const std::vector<sensor_msgs::msg::Image> masks_msg_vec)
{
    pre_process(masks_msg_vec);
}

void MaskImages::msg_mask_to_binary(const std::vector<sensor_msgs::msg::Image> masks_msg_vec)
{
    size_t masks_s = masks_msg_vec.size();
    cv_bin_mask_vec_.resize(masks_s);
    sensor_msgs::msg::Image::SharedPtr mask_msg_ptr;
    for(size_t i=0; i<masks_s; ++i){
        mask_msg_ptr.reset(new sensor_msgs::msg::Image(masks_msg_vec[i]));
        img_msg_to_cv(mask_msg_ptr, cv_bin_mask_vec_[i]);
    }
}

void MaskImages::add_weight_bin(void)
{
    cv_aw_bin_mask_ = cv::Mat::zeros(cv_bin_mask_vec_[0].rows, cv_bin_mask_vec_[0].cols, cv_bin_mask_vec_[0].type());
    for(auto &bin: cv_bin_mask_vec_){
        bin.forEach<u_char>([&](uchar &pixel, const int position[]) -> void{
            if (pixel > 0) {
                cv_aw_bin_mask_.at<uint8_t>(position[0], position[1]) = 1;
            }
        });
    }
}

void MaskImages::bin_mask_to_rgb(void)
{
    cv::Vec3b color(255, 255, 255);
    cv_raw_rgb_mask_ = cv::Mat::zeros(cv_aw_bin_mask_.rows, cv_aw_bin_mask_.cols, CV_8UC3);
    cv_aw_bin_mask_.forEach<uchar>([&](uchar &pixel, const int position[]) -> void {
        if (pixel > 0) {
            cv_raw_rgb_mask_.at<cv::Vec3b>(position[0], position[1]) = color;
        }
    });
    // cv_inv_rgb_mask_ = ~(cv_raw_rgb_mask_.clone());
}

void MaskImages::find_contours_process(
    cv::Mat base, 
    std::vector<std::vector<cv::Point>> & output)
{
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat gray, binary;
    cv::cvtColor(base, gray, CV_RGB2GRAY);
    cv::threshold(gray, binary, 150, 255, cv::THRESH_BINARY);
    cv::findContours(binary, output, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
}

void MaskImages::generate_contours_point(void)
{
    find_contours_process(cv_raw_rgb_mask_, raw_contours_); // 生のRGBマスク：輪郭抽出
    find_contours_process(~cv_raw_rgb_mask_, inv_contours_); // 反転RGBマスク：輪郭抽出
    remove_contours_noise(); // 反転RGBマスク輪郭：輪郭面積が閾値以上のものは削除
    find_contours_process(cv_rgb_mask_, contours_); // cv_rgb_mask_：輪郭抽出
}

void MaskImages::remove_contours_noise(void)
{
    std::vector<std::vector<cv::Point>> noise_contours;
	for(auto & c: inv_contours_){
        if(noise_contour_area_th_ > cv::contourArea(c)){
            noise_contours.emplace_back(c);
        }
    }
    cv_rgb_mask_ = cv_raw_rgb_mask_.clone();
    cv::drawContours(cv_rgb_mask_, noise_contours, -1, cv::Scalar(255, 255, 255), -1);
    RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Complete Remove Contours Noise");
}

void MaskImages::get_bin_mask(cv::Mat & output)
{
    output = cv_aw_bin_mask_.clone();
}

void MaskImages::get_contours(std::vector<std::vector<cv::Point>> & output)
{
    output = contours_;
}

void MaskImages::get_image_size(int & rows, int & cols)
{
    rows = cv_rgb_mask_.rows;
    cols = cv_rgb_mask_.cols;
}

void MaskImages::draw_mask_contours_bbox(cv::Mat & base, bool raw)
{
    if(raw){
        cv::addWeighted(base, 1.0, cv_raw_rgb_mask_, 0.5, 0.0, base);
        cv::drawContours(base, raw_contours_, -1, cv::Scalar(255, 0, 0), 5);
    }else{
        cv::addWeighted(base, 1.0, cv_rgb_mask_, 0.5, 0.0, base);
        cv::drawContours(base, contours_, -1, cv::Scalar(255, 0, 0), 5);
    }
}

MaskImages::~MaskImages(){}

}
