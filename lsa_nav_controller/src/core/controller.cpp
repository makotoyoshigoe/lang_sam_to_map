// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include <cmath>

#include "lsa_nav_controller/core/controller.hpp"

namespace lsa_nav_controller
{


Controller::Controller(
    float lin_max_vel, float lin_min_vel, float ang_max_vel, float ang_min_vel, 
    float lin_acc_th, float lin_dec_th, float ang_acc_th, float ang_dec_th, 
    float kp, float ki, float kd, float rate)
: lin_max_vel_(lin_max_vel), lin_min_vel_(lin_min_vel), ang_max_vel_(ang_max_vel), ang_min_vel_(ang_min_vel), 
  lin_acc_th_(lin_acc_th), lin_dec_th_(lin_dec_th), ang_acc_th_(ang_acc_th), ang_dec_th_(ang_dec_th), 
  kp_(kp), ki_(ki), kd_(kd), rate_(rate), ei_(0.), pre_e_(0.), pre_cmd_{0., 0.}
{
    ei_ = 0.;
    init_ei_ = false;
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), "ki: %f, ei: %.3f, rate: %f", ki_, ei_, rate_);
}

CmdVel Controller::get_cmd_vel(
    float ave_right, float ave_left, float ave_front)
{
    CmdVel cur_cmd;
    cur_cmd.lin_vel = std::max(std::min(get_lin_vel(ave_front), lin_max_vel_), lin_min_vel_);
    cur_cmd.ang_vel = std::max(std::min(get_ang_vel(ave_right, ave_left), ang_max_vel_), ang_min_vel_);
    cur_cmd = smooth_vel(cur_cmd);
    pre_cmd_ = cur_cmd;
    return cur_cmd;
}

CmdVel Controller::smooth_vel(CmdVel raw_cmd)
{
    float diff_lin_vel = raw_cmd.lin_vel - pre_cmd_.lin_vel;
    float diff_ang_vel = raw_cmd.ang_vel - pre_cmd_.ang_vel;
    return {
        diff_lin_vel > lin_acc_th_ * rate_ ? pre_cmd_.lin_vel + lin_acc_th_ * rate_: raw_cmd.lin_vel, 
        diff_ang_vel > ang_acc_th_ * rate_ ? pre_cmd_.ang_vel + ang_acc_th_ * rate_: raw_cmd.ang_vel
    };
}

float Controller::get_lin_vel(float ave_front)
{
    return lin_max_vel_;
}

float Controller::get_ang_vel(float ave_right, float ave_left)
{
    float goal = (ave_left + ave_right) / 2;
    float e = ave_left - goal;
    if(std::isnan(ei_)) ei_ = 0.;
    ei_ += e * rate_;
    float ed = (e - pre_e_) / rate_;
    pre_e_ = e;
    RCLCPP_INFO(rclcpp::get_logger("lsa_nav_controller"), 
        "goal: %f, e: %f, kp*e: %.3f, right: %.3f, left: %.3f", goal, e, kp_*e, ave_right, ave_left);
    return kp_ * e + ki_ * ei_ + kd_ * ed;
}

Controller::~Controller(){}
    
} // namespace lsa_nav_controller
