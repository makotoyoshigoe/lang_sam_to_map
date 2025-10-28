// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "lang_sam_to_map/lang_sam_to_map.hpp"
#include "lang_sam_nav_msgs/srv/get_map.hpp"

namespace lang_sam_to_map{
class LangSamToMapServer : public LangSamToMap{
public:
	LangSamToMapServer(std::string node_name);
	~LangSamToMapServer();
    void init_get_map_server(void);
    void cb_get_map(
        const std::shared_ptr<lang_sam_nav_msgs::srv::GetMap::Request> request,
        std::shared_ptr<lang_sam_nav_msgs::srv::GetMap::Response> response);

private:
    rclcpp::Service<lang_sam_nav_msgs::srv::GetMap>::SharedPtr srv_get_map_;
};
}
