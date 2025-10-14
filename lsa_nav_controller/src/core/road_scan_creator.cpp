// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/core/road_scan_creator.hpp"

namespace lsa_nav_controller
{

RoadScanCreator::RoadScanCreator(
    float max_angle_abs, float min_angle_abs, float angle_increment, float max_range, float min_range)
: max_angle_abs_(max_angle_abs), min_angle_abs_(min_angle_abs), angle_increment_(angle_increment), 
  max_range_(max_range), min_range_(min_range), 
  Map()
{
    
} 

bool RoadScanCreator::create_road_scan()
{
    return true;
}


RoadScanCreator::~RoadScanCreator(){}
    
} // namespace lsa_nav_controller
