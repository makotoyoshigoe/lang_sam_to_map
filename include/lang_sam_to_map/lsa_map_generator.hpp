// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/map.hpp"
#include "lang_sam_to_map/depth_image.hpp"
#include "lang_sam_to_map/mask_images.hpp"

namespace lang_sam_to_map{
class LSAMapGenerator : public lang_sam_to_map::Map{
    public:
    LSAMapGenerator(
        std::vector<sensor_msgs::msg::Image> & masks_msg_vec, 
        sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg, 
        std::string frame_id, float resolution, int width, int height);
    ~LSAMapGenerator();

    private:
    std::vector<sensor_msgs::msg::Image> masks_msg_vec_;
    std::unique_ptr<MaskImages> mask_images_;
    std::unique_ptr<DepthImage> depth_image_;
};

} // namespace lang_sam_to_map

