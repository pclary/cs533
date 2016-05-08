#pragma once

#include "State.hpp"

namespace sim
{


struct ControllerTarget
{
};


struct ControllerParams
{
};


struct MotorTorques
{
    double length;
    double angle;
};


MotorTorques low_level_controller(State state,
                                  double t,
                                  ControllerTarget target,
                                  ControllerParams params);


} // namespace sim
