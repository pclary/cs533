#include "sim/sim.hpp"
#include "vis/vis.hpp"


int main()
{
    // Load simulation parameters
    sim::Environment env;
    sim::load(env, "environment.txt");

    // Initial state
    sim::State initial = {0.0, 0.7, 0.0, 0.7, 0.7, 0.3, 0.3,
                          1.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Simulate until first touchdown
    sim::StateSeries ss = sim::simulate_hopper(initial, 1.0, env, {}, {});

    // Simulate step by step until the window is closed
    vis::Hopper hopper(env);
    while (hopper.isAlive())
    {
        ss = sim::simulate_hopper(ss.back().state, 3.0, env, {1}, {});
        hopper.animate(ss);
    }
}
