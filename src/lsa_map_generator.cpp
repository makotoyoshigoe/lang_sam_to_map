// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/lsa_map_generator.hpp"

namespace lang_sam_to_map
{

LSAMapGenerator::LSAMapGenerator(
    std::vector<sensor_msgs::msg::Image> & masks_msg_vec, 
    sensor_msgs::msg::Image::ConstSharedPtr depth_msg, 
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg, 
    std::string frame_id, float resolution, int width, int height)
: Map(frame_id, resolution, width, height)
{
    mask_images_.reset(new MaskImages(masks_msg_vec));
    depth_image_.reset(new DepthImage(depth_msg, camera_info_msg));
}



LSAMapGenerator::~LSAMapGenerator(){}
    
} // namespace lang_sam_to_map
