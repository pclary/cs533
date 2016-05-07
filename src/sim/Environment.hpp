#pragma once

#include <vector>

namespace sim
{


struct GroundVertex
{
    double x;
    double y;
    double stiffness;
    double damping;
    double friction;
};


struct Environment
{
    double mass;
    double inertia;
    double foot_mass;
    double length_stiffness;
    double length_damping;
    double length_motor_inertia;
    double length_motor_ratio;
    double angle_stiffness;
    double angle_damping;
    double angle_motor_inertia;
    double angle_motor_ratio;
    double gravity;
    std::vector<GroundVertex> ground;
};


} // namespace sim
