// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/sensor/mask_images.hpp"

namespace lang_sam_to_map{
MaskImages::MaskImages(const std::vector<sensor_msgs::msg::Image> masks_msg_vec) : Image()
{
    msg_mask_to_binary(masks_msg_vec);
    add_weight_bin();
    bin_mask_to_rgb();
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
    cv_rgb_mask_ = cv::Mat::zeros(cv_aw_bin_mask_.rows, cv_aw_bin_mask_.cols, CV_8UC3);
    cv_aw_bin_mask_.forEach<uchar>([&](uchar &pixel, const int position[]) -> void {
        if (pixel > 0) {
            cv_rgb_mask_.at<cv::Vec3b>(position[0], position[1]) = color;
        }
    });
}

void MaskImages::find_contours(std::vector<std::vector<cv::Point>> & contours)
{
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat gray, binary;
    cv::cvtColor(cv_rgb_mask_, gray, CV_RGB2GRAY);
    cv::threshold(gray, binary, 150, 255, cv::THRESH_BINARY);
    cv::findContours(binary, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
}

void MaskImages::get_bin_mask(cv::Mat & output)
{
    output = cv_aw_bin_mask_.clone();
}

cv::Mat MaskImages::vis_mask_contours_bbox(
    sensor_msgs::msg::Image::ConstSharedPtr base, 
    std::vector<sensor_msgs::msg::RegionOfInterest> & boxes)
{
    cv::Mat cv_vis;
    img_msg_to_cv(base, cv_vis);
    cv::addWeighted(cv_vis, 1.0, cv_rgb_mask_, 0.5, 0.0, cv_vis);
    for(auto &c: contours_) cv::drawContours(cv_vis, c, -1, cv::Scalar(255, 0, 0), 1);
	for(auto &box: boxes){
		// バウンディングボックスを描画
		cv::rectangle(cv_vis, 
            cv::Point(box.x_offset, box.y_offset), 
            cv::Point(box.x_offset+box.width, box.y_offset+box.height), 
            cv::Scalar(0, 0, 255), 1);
	}
    return cv_vis;
}

MaskImages::~MaskImages(){}

}
