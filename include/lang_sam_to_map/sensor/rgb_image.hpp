// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/sensor/image.hpp"

namespace lang_sam_to_map{
class RGBImage : public Image
{
    public:
    RGBImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
    RGBImage(void);
    ~RGBImage();
    cv::Vec3b get_pixel_color(int u, int v);
    void get_image_size(int & rows, int & cols);
    void set_image(cv::Mat image);

    private:
    cv::Mat cv_color_;
};

}
