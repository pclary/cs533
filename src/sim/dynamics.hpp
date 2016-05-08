#pragma once

#include "State.hpp"
#include "Environment.hpp"

namespace sim
{


State simulate_hopper(State initial, Environment env, double time = 0.0);


} // namespace sim
