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


StateSeries simulate_hopper(State initial,
                            double sim_time,
                            Environment env,
                            ControllerTarget target,
                            ControllerParams params);


} // namespace sim
