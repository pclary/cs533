#pragma once

#include "State.hpp"
#include "Environment.hpp"
#include "controller.hpp"
#include <vector>

namespace sim
{


StateSeries simulate_hopper(State initial, double sim_time, Environment env,
                            ControllerTarget target, ControllerParams params);


} // namespace sim
