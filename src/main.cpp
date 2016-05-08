#include <iostream>
#include <vector>
#include "sim/sim.hpp"


int main()
{
    // Test serialization functions

    // Save states
    sim::StateSeries states;
    for (int i = 0; i < 100; ++i)
        states.push_back({i+0.0, {i+0.01, i+0.02, i+0.03, i+0.04, i+0.05, i+0.06, i+0.07, i+0.08, i+0.09, i+0.10, i+0.11, i+0.12, i+0.13, i+0.14}});
    sim::save(states, "states.txt");

    // Save environment
    sim::Environment env;
    env.body_mass = 0.01;
    env.body_inertia = 0.02;
    env.foot_mass = 0.03;
    env.length_stiffness = 0.04;
    env.length_damping = 0.05;
    env.length_motor_inertia = 0.06;
    env.length_motor_damping = 0.07;
    env.length_motor_ratio = 0.08;
    env.length_min = 0.09;
    env.length_max = 0.10;
    env.angle_stiffness = 0.11;
    env.angle_damping = 0.12;
    env.angle_motor_inertia = 0.13;
    env.angle_motor_damping = 0.14;
    env.angle_motor_ratio = 0.15;
    env.angle_min = 0.16;
    env.angle_max = 0.17;
    env.gravity = 0.18;
    for (int i = 0; i < 50; ++i)
        env.ground.push_back({i+0.1, i+0.2, i+0.3, i+0.4, i+0.5});
    env.dt = 0.19;
    env.length_hardstop_kp = 0.20;
    env.length_hardstop_kd = 0.21;
    env.length_hardstop_dfade = 0.22;
    env.length_hardstop_fmax = 0.23;
    env.angle_hardstop_kp = 0.24;
    env.angle_hardstop_kd = 0.25;
    env.angle_hardstop_dfade = 0.26;
    env.angle_hardstop_fmax = 0.27;
    env.ground_damping_depth = 0.28;
    env.ground_slip_ramp = 0.29;
    sim::save(env, "environment.txt");

    // Load and save again
    sim::StateSeries states2;
    sim::load(states2, "states.txt");
    sim::save(states2, "states2.txt");

    sim::Environment env2;
    sim::load(env, "environment.txt");
    sim::save(env, "environment2.txt");
}
