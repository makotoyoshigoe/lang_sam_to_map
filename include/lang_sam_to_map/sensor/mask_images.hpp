// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/sensor/image.hpp"
#include <sensor_msgs/msg/region_of_interest.hpp>

namespace lang_sam_to_map{

class MaskImages : public Image
{
    public:
    MaskImages(const std::vector<sensor_msgs::msg::Image> masks_msg_vec);
    MaskImages(float noise_contour_th_);
    ~MaskImages();
    void pre_process(const std::vector<sensor_msgs::msg::Image> masks_msg_vec);
    void set_mask(const std::vector<sensor_msgs::msg::Image> masks_msg_vec);
    void msg_mask_to_binary(const std::vector<sensor_msgs::msg::Image> masks_msg_vec);
    void add_weight_bin(void);
    void bin_mask_to_rgb(void);
    void find_contours_process(
        cv::Mat base, std::vector<std::vector<cv::Point>> & output);
    void generate_contours_point(void);
	void remove_contours_noise(void);
    void get_bin_mask(cv::Mat & output);
    void get_contours(std::vector<std::vector<cv::Point>> & output);
    void get_image_size(int & rows, int & cols);
    void draw_mask_contours_bbox(cv::Mat & base, bool raw);

    private:
    float noise_contour_th_;
    std::vector<cv::Mat> cv_bin_mask_vec_;
    cv::Mat cv_aw_bin_mask_;
    cv::Mat cv_raw_rgb_mask_, cv_inv_rgb_mask_, cv_rgb_mask_;
    std::vector<std::vector<cv::Point>> raw_contours_, inv_contours_, contours_;
};

}
