#include "sim/sim.hpp"


int main()
{
    // Load simulation parameters
    sim::Environment env;
    sim::load(env, "environment.txt");

    // Initial state
    sim::State initial = {0.0, 0.7, 0.0, 0.7, 0.7, 0.0, 0.0,
                          0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Simulate for a while
    sim::StateSeries ss = sim::simulate_hopper(initial, 10.0, env, {}, {});

    // Save output
    sim::save(ss, "statedata.txt");
}
