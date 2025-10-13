// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/lsa_nav_controller.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lsa_nav_controller::LsaNavController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}