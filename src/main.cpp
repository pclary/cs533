#include "sim/sim.hpp"
#include "vis/vis.hpp"
#include <iostream>


int main()
{
    // Load simulation parameters
    sim::Environment env;
    sim::load(env, "environment.txt");

    // Initial state
    sim::State initial = {0.0, 0.8, 0.0, 0.4, 0.0, 0.0,
                          {0.8, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                          {0.7, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    // Simulate until first touchdown
    sim::ControllerState cstate;
    sim::StateSeries ss =
        sim::simulate_hopper(initial, 0.0, env, {}, {}, cstate);

    // Simulate step by step until the window is closed
    vis::Biped biped(env);
    while (biped.isAlive())
    {
        std::cout << ss.back().state.dx << " " << ss.back().state.leg_a.l_eq << std::endl;
        ss = sim::simulate_hopper(ss.back().state, 3.0, env, {0.8}, {}, cstate);
        biped.animate(ss);
    }
}
