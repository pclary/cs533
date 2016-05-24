#pragma once

#include "State.hpp"
#include "Environment.hpp"
#include <vector>

namespace sim
{


struct ControllerTarget
{
    double velocity; // typical: -3 to 3, higher if already moving quickly
};


struct ControllerParams
{
    double leg_extension;   // typical: 0 to 0.1
    double horizontal_push; // typical: -1000 to 1000
};


struct TrajectoryValues
{
    double phase;
    double torque;
    double dtorque;
    double target;
    double dtarget;
    double kp;
    double kd;
};


struct ControllerState
{
    double phase_a = 0.0;
    double phase_b = 0.5;

    TrajectoryValues length_tvals_a;
    TrajectoryValues angle_tvals_a;
    TrajectoryValues body_angle_tvals_a;
    TrajectoryValues length_tvals_b;
    TrajectoryValues angle_tvals_b;
    TrajectoryValues body_angle_tvals_b;

    double dx_midstance = 0.0;

    bool end_step = false;
};


StateSeries simulate_hopper(State initial,
                            double sim_time,
                            Environment env,
                            ControllerTarget target,
                            ControllerParams params,
                            ControllerState& cstate);


} // namespace sim
