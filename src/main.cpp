#include <iostream>
#include <vector>
#include "sim/sim.hpp"


int main()
{
    // Test serialization functions

    // Save states
    std::vector<sim::State> states;
    for (int i = 0; i < 100; ++i)
        states.push_back({i+0.01, i+0.02, i+0.03, i+0.04, i+0.05, i+0.06, i+0.07, i+0.08, i+0.09, i+0.10, i+0.11, i+0.12});
    sim::save(states, "states.txt");

    // Save environment
    sim::Environment env;
    env.mass = 0.01;
    env.inertia = 0.02;
    env.foot_mass = 0.03;
    env.length_stiffness = 0.04;
    env.length_damping = 0.05;
    env.length_motor_inertia = 0.06;
    env.length_motor_ratio = 0.07;
    env.angle_stiffness = 0.08;
    env.angle_damping = 0.09;
    env.angle_motor_inertia = 0.10;
    env.angle_motor_ratio = 0.11;
    env.gravity = 0.12;
    for (int i = 0; i < 50; ++i)
        env.ground.push_back({i+0.1, i+0.2, i+0.3, i+0.4, i+0.5});
    sim::save(env, "environment.txt");

    // Load and save again
    std::vector<sim::State> states2;
    sim::load(states2, "states.txt");
    sim::save(states2, "states2.txt");

    sim::Environment env2;
    sim::load(env, "environment.txt");
    sim::save(env, "environment2.txt");
}
