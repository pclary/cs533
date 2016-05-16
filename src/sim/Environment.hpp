#pragma once

#include <vector>

namespace sim
{


struct GroundData
{
    double x;         // Horizontal location of this ground vertex
    double y;         // Vertical location of this ground vertex
    double stiffness; // Ground stiffnes at this vertex
    double damping;   // Ground damping
    double friction;  // Kinetic friction factor, between 0 and 1
};


struct Environment
{
    double body_mass;             // Total mass of the body, including motors
    double body_inertia;          // Moment of inertia of the body
    double foot_mass;             // Mass of the point foot

    double length_stiffness;      // Stiffness of the leg length spring
    double length_damping;        // Damping of the leg length spring
    double length_motor_inertia;  // Moment of inertia of the leg length motor
    double length_motor_damping;  // Damping in the leg length motor + gearbox
    double length_motor_ratio;    // Gear reduction, should be greater than 1
    double length_min;            // Leg length lower limit hardstop
    double length_max;            // Leg length upper limit hardstop

    double angle_stiffness;       // Stiffness of the leg angle spring
    double angle_damping;         // Damping of the leg angle spring
    double angle_motor_inertia;   // Moment of inertia of the leg angle motor
    double angle_motor_damping;   // Damping in the leg angle motor + gearbox
    double angle_motor_ratio;     // Factor for converting from length to angle
    double angle_min;             // Leg angle lower limit hardstop
    double angle_max;             // Leg angle upper limit hardstop

    double gravity;               // Gravitational acceleration
    std::vector<GroundData> ground; // The ground defined as a polyline

    double dt;                    // Simulation timestep
    double length_hardstop_kp;    // Proportional constant for length hardstop
    double length_hardstop_kd;    // Derivative constant for length hardstop
    double length_hardstop_dfade; // Width of derivative fade region
    double length_hardstop_fmax;  // Max hardstop force
    double angle_hardstop_kp;     // Proportional constant for angle hardstop
    double angle_hardstop_kd;     // Derivative constant for angle hardstop
    double angle_hardstop_dfade;  // Width of derivative fade region
    double angle_hardstop_fmax;   // Max hardstop force
    double ground_damping_depth;  // Fade-in depth for ground damping
    double ground_slip_ramp;      // Factor used to smooth friction model
};


} // namespace sim
