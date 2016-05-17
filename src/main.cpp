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
    sim::StateSeries ss = {{0.0, initial}};

    // Simulate several steps
    vis::Hopper hopper(env);
    while (hopper.isAlive())
    {
        ss = sim::simulate_hopper(ss.back().state, 5.0, env, {1}, {});
        hopper.animate(ss);
    }

    // Save output
    // sim::save(ss, "statedata.txt");
}
