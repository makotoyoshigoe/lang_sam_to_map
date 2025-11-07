// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/sensor/rgb_image.hpp"
#include "lang_sam_to_map/sensor/depth_image.hpp"

namespace lang_sam_to_map{
class RGBDImage : public lang_sam_to_map::RGBImage, public lang_sam_to_map::DepthImage
{
    public:
    RGBDImage(float min_depth_th, uint8_t depth_correct_range);
    ~RGBDImage();
    
    private:
};

}
