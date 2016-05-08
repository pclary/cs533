#pragma once

#include <vector>

namespace sim
{


struct GroundVertex
{
    double x;         // Horizontal location of this ground vertex
    double y;         // Vertical location of this ground vertex
    double stiffness; // Ground stiffnes at this vertex
    double damping;   // Ground damping
    double friction;  // Kinetic friction factor, between 0 and 1
};


struct Environment
{
    double body_mass;            // Total mass of the body, including motors
    double body_inertia;         // Moment of inertia of the body
    double foot_mass;            // Mass of the point foot
    double length_stiffness;     // Stiffness of the leg length spring
    double length_damping;       // Damping of the leg length spring
    double length_motor_inertia; // Moment of inertia of the leg length motor
    double length_motor_damping; // Damping in the leg length motor + gearbox
    double length_motor_ratio;   // Gear reduction, should be less than 1
    double angle_stiffness;      // Stiffness of the leg angle spring
    double angle_damping;        // Damping of the leg angle spring
    double angle_motor_inertia;  // Moment of inertia of the leg angle motor
    double angle_motor_damping;  // Damping in the leg angle motor + gearbox
    double angle_motor_ratio;    // Factor for converting from angle to length
    double gravity;              // Gravitational acceleration
    std::vector<GroundVertex> ground; // The ground defined as a polyline
};


} // namespace sim
