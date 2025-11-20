// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <lsa_nav_controller/utils/scan_utils.hpp>

namespace scan_utils
{
float index_to_rad(
    size_t index, float min_angle, float angle_increment)
{return index * angle_increment + min_angle;}

size_t rad_to_index(
    float rad, float min_angle, float angle_increment)
{return (rad - min_angle) / angle_increment;}

} // namespace scan_utils