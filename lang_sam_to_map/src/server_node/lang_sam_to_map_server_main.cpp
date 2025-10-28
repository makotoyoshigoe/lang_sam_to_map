// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/server_node/lang_sam_to_map_server.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lang_sam_to_map::LangSamToMapServer>("lang_sam_to_map_server");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}