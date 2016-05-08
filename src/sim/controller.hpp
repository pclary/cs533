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


// For convenience when combining terms, may add other operators if needed
inline MotorTorques operator+ (const MotorTorques& a, const MotorTorques& b)
{
    return {a.length + b.length, a.angle + b.angle};
}


} // namespace sim
