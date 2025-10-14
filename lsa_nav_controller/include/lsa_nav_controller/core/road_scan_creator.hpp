// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lsa_nav_controller/map/map.hpp"

namespace lsa_nav_controller
{
class RoadScanCreator : public Map{
    public:
    RoadScanCreator(float max_angle_abs, float min_angle_abs, float angle_increment, float max_range, float min_range);
    ~RoadScanCreator();
    bool create_road_scan();

    private:
    float max_angle_abs_, min_angle_abs_, angle_increment_;
    float max_range_, min_range_;
};
    
} // namespace lsa_nav_controller
