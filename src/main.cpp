// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/lang_sam_to_map.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lang_sam_to_map::LangSamToMap>();
    rclcpp::Rate loop_rate(node->get_node_freq());
    while (rclcpp::ok()){
        if(node->flg_send_request()){
            RCLCPP_INFO(rclcpp::get_logger("lang_sam_to_map"), "Waiting for Response");
            node->send_request();
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}