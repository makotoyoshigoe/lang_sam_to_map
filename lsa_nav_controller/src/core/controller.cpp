// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#include "lsa_nav_controller/core/controller.hpp"

namespace lsa_nav_controller
{


Controller::Controller(
    float lin_max_vel, float lin_min_vel, float ang_max_vel, float ang_min_vel, 
    float lin_max_acc, float lin_min_acc, float ang_max_acc, float ang_min_acc)
: lin_max_vel_(lin_max_vel), lin_min_vel_(lin_min_vel), ang_max_vel_(ang_max_vel), ang_min_vel_(ang_min_vel), 
  lin_max_acc_(lin_max_acc), lin_min_acc_(lin_min_acc), ang_max_acc_(ang_max_acc), ang_min_acc_(ang_min_acc) 
{

}

CmdVel Controller::get_cmd_vel(
    float ave_right, float ave_left, float ave_front)
{
    return {(ave_right + ave_left) / 2, ave_front};
}

Controller::~Controller(){}
    
} // namespace lsa_nav_controller
