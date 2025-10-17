// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com 
// SPDX-License-Identifier: Apache-2.0

#pragma once

namespace lsa_nav_controller
{
class Controller{
    public:
    Controller(
        float lin_max_vel, float lin_min_vel, float ang_max_vel, float ang_min_vel, 
        float lin_max_acc, float lin_min_acc, float ang_max_acc, float ang_min_acc);
    
    ~Controller();

    private:
    float lin_max_vel_, lin_min_vel_, ang_max_vel_, ang_min_vel_;
    float lin_max_acc_, lin_min_acc_, ang_max_acc_, ang_min_acc_;
};
    
} // namespace lsa_nav_controller
