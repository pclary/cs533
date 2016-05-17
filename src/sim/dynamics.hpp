#pragma once

#include "State.hpp"
#include "Environment.hpp"
#include <vector>

namespace sim
{


struct ControllerTarget
{
    double velocity;
};


struct ControllerParams
{
    double leg_extension;
    double horizontal_push;
};


StateSeries simulate_hopper(State initial,
                            double sim_time,
                            Environment env,
                            ControllerTarget target,
                            ControllerParams params);


} // namespace sim
