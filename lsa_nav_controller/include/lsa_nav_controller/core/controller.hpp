// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <vector>

namespace lsa_nav_controller
{
struct CmdVel
{
    float lin_vel;
    float ang_vel;
};

class Controller{
    public:
    Controller(
        float lin_max_vel, float lin_min_vel, float ang_max_vel, float ang_min_vel, 
        float lin_acc_th, float lin_dec_th, float ang_acc_th, float ang_dec_th, 
        float kp, float ki, float kd, float rate);
    CmdVel get_cmd_vel(float ave_right, float ave_left, float ave_front);
    float get_ang_vel(float ave_right, float ave_left);
    float get_lin_vel(float ave_front);
    CmdVel smooth_vel(CmdVel raw_cmd);

    ~Controller();

    private:
    float lin_max_vel_, lin_min_vel_, ang_max_vel_, ang_min_vel_;
    float lin_acc_th_, lin_dec_th_, ang_acc_th_, ang_dec_th_;
    float kp_, ki_, kd_, rate_;
    float ei_, pre_e_;
    CmdVel pre_cmd_;
    bool init_ei_;
};
    
} // namespace lsa_nav_controller
