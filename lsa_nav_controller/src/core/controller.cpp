// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/core/controller.hpp"

namespace lsa_nav_controller
{


Controller::Controller(
    float lin_max_vel, float lin_min_vel, float ang_max_vel, float ang_min_vel, 
    float lin_max_acc, float lin_min_acc, float ang_max_acc, float ang_min_acc, 
    float kp, float ki, float kd, float rate)
: lin_max_vel_(lin_max_vel), lin_min_vel_(lin_min_vel), ang_max_vel_(ang_max_vel), ang_min_vel_(ang_min_vel), 
  lin_max_acc_(lin_max_acc), lin_min_acc_(lin_min_acc), ang_max_acc_(ang_max_acc), ang_min_acc_(ang_min_acc), 
  kp_(kp), ki_(ki), kd_(kd), rate_(rate), ei_(0.), pre_e_(0.), pre_cmd_{0., 0.}
{

}

CmdVel Controller::get_cmd_vel(
    float ave_right, float ave_left, float ave_front)
{
    float lin_vel = lin_max_vel_;
    float ang_vel = get_ang_vel(ave_right, ave_left);
    pre_cmd_.ang_vel = ang_vel;
    return {lin_vel, std::min(ang_vel, lin_max_vel_)};
}

CmdVel Controller::smooth_vel(CmdVel raw_cmd)
{
    float diff_lin = raw_cmd.lin_vel - pre_cmd_.lin_vel;
    float diff_ang = raw_cmd.ang_vel - pre_cmd_.ang_vel;
    CmdVel cmd;
    cmd.lin_vel = (diff_lin / rate_ > lin_max_acc_ ? pre_cmd_.lin_vel + lin_max_acc_ * rate_: raw_cmd.lin_vel);
    return {diff_lin / rate_, diff_ang / rate_};
}

float Controller::get_ang_vel(float ave_right, float ave_left)
{
    float goal = (ave_left + ave_right) / 2;
    float d = ave_left - ave_right;
    float e = d - goal;
    ei_ += e * rate_;
    float ed = (e - pre_e_) / rate_;
    return kp_ * e + ki_ * ei_ + kd_ * ed;
}

Controller::~Controller(){}
    
} // namespace lsa_nav_controller
