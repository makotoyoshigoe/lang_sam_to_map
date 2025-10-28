// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lang_sam_to_map/server_node/lang_sam_to_map_server.hpp"

namespace lang_sam_to_map{
LangSamToMapServer::LangSamToMapServer(std::string node_name)
 : LangSamToMap(node_name) 
{
    init_get_map_server();
}

void LangSamToMapServer::init_get_map_server(void)
{
    srv_get_map_ = this->create_service<lang_sam_nav_msgs::srv::GetMap>(
        "get_lang_sam_map",
        std::bind(&LangSamToMapServer::cb_get_map, this, std::placeholders::_1, std::placeholders::_2));
}

void LangSamToMapServer::cb_get_map(
    const std::shared_ptr<lang_sam_nav_msgs::srv::GetMap::Request> request,
    std::shared_ptr<lang_sam_nav_msgs::srv::GetMap::Response> response)
{
    send_request();
    nav_msgs::msg::OccupancyGrid lang_sam_map;
    lsa_map_generator_->get_map_msg(lang_sam_map);
    response->map = lang_sam_map;
    RCLCPP_INFO(get_logger(), "Get Map Service was Called");
}

LangSamToMapServer::~LangSamToMapServer(){}

} // namespace lang_sam_to_map
