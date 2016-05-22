#include "io.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <exception>

namespace sim
{


// Convenience macros
#define WRITE(obj, stream, name) do {                                   \
        stream << std::setw(24) << std::left;                           \
        stream << #name ": " << obj.name << std::endl;                  \
    } while (0)
#define READ(obj, stream, name) do {                                    \
        std::string label;                                              \
        stream >> label >> obj.name;                                    \
        if (label != #name ":")                                         \
            throw std::runtime_error("Bad input file format at " #name);\
    } while (0)


void save(const Environment& env, std::string filename)
{
    // Open file for writing
    std::ofstream file(filename);

    // Write values from Environment structure
    file << "# Mass properties" << std::endl;
    WRITE(env, file, body_mass);
    WRITE(env, file, body_inertia);
    WRITE(env, file, foot_mass);

    file << std::endl << "# Leg length parameters" << std::endl;
    WRITE(env, file, length_stiffness);
    WRITE(env, file, length_damping);
    WRITE(env, file, length_motor_inertia);
    WRITE(env, file, length_motor_damping);
    WRITE(env, file, length_motor_ratio);
    WRITE(env, file, length_min);
    WRITE(env, file, length_max);

    file << std::endl << "# Leg angle parameters" << std::endl;
    WRITE(env, file, angle_stiffness);
    WRITE(env, file, angle_damping);
    WRITE(env, file, angle_motor_inertia);
    WRITE(env, file, angle_motor_damping);
    WRITE(env, file, angle_motor_ratio);
    WRITE(env, file, angle_min);
    WRITE(env, file, angle_max);

    file << std::endl << "# Environmental data" << std::endl;
    WRITE(env, file, gravity);

    // Write ground vertex data
    file << "ground: ";
    file << "[x          y          stiffness  damping    friction  ]";
    file << std::endl;
    for (const auto v : env.ground)
    {
        const int w = 10;
        file << "         ";
        file << std::setw(w) << v.x         << " ";
        file << std::setw(w) << v.y         << " ";
        file << std::setw(w) << v.stiffness << " ";
        file << std::setw(w) << v.damping   << " ";
        file << v.friction  << std::endl;
    }

    file << std::endl << "# Simulation details" << std::endl;
    WRITE(env, file, dt);
    WRITE(env, file, length_hardstop_kp);
    WRITE(env, file, length_hardstop_kd);
    WRITE(env, file, length_hardstop_dfade);
    WRITE(env, file, length_hardstop_fmax);
    WRITE(env, file, angle_hardstop_kp);
    WRITE(env, file, angle_hardstop_kd);
    WRITE(env, file, angle_hardstop_dfade);
    WRITE(env, file, angle_hardstop_fmax);
    WRITE(env, file, ground_damping_depth);
    WRITE(env, file, ground_slip_ramp);
}


void save(const StateSeries& states, std::string filename)
{
    // Open file for writing
    std::ofstream file(filename);
    file.precision(4);

    // Write states from vector
    file << "t       x           y           phi         dx          dy          dphi        l           l_eq        theta       theta_eq    dl          dl_eq       dtheta      dtheta_eq   l           l_eq        theta       theta_eq    dl          dl_eq       dtheta      dtheta_eq" << std::endl;
    for (const auto ts : states)
    {
        const int w = 11;
        file << std::fixed;
        file << std::setw(6) << ts.time                  << " ";
        file << std::scientific;
        file << std::setw(w) << ts.state.x               << " ";
        file << std::setw(w) << ts.state.y               << " ";
        file << std::setw(w) << ts.state.phi             << " ";
        file << std::setw(w) << ts.state.dx              << " ";
        file << std::setw(w) << ts.state.dy              << " ";
        file << std::setw(w) << ts.state.dphi            << " ";
        file << std::setw(w) << ts.state.leg_a.l         << " ";
        file << std::setw(w) << ts.state.leg_a.l_eq      << " ";
        file << std::setw(w) << ts.state.leg_a.theta     << " ";
        file << std::setw(w) << ts.state.leg_a.theta_eq  << " ";
        file << std::setw(w) << ts.state.leg_a.dl        << " ";
        file << std::setw(w) << ts.state.leg_a.dl_eq     << " ";
        file << std::setw(w) << ts.state.leg_a.dtheta    << " ";
        file << std::setw(w) << ts.state.leg_a.dtheta_eq << " ";
        file << std::setw(w) << ts.state.leg_b.l         << " ";
        file << std::setw(w) << ts.state.leg_b.l_eq      << " ";
        file << std::setw(w) << ts.state.leg_b.theta     << " ";
        file << std::setw(w) << ts.state.leg_b.theta_eq  << " ";
        file << std::setw(w) << ts.state.leg_b.dl        << " ";
        file << std::setw(w) << ts.state.leg_b.dl_eq     << " ";
        file << std::setw(w) << ts.state.leg_b.dtheta    << " ";
        file << std::setw(w) << ts.state.leg_b.dtheta_eq << std::endl;
    }
}


void load(Environment& env, std::string filename)
{
    // Open file for reading
    std::ifstream file(filename);
    std::string line;

    // Read values into Environment structure
    std::getline(file, line); // Skip section header
    READ(env, file, body_mass);
    READ(env, file, body_inertia);
    READ(env, file, foot_mass);

    std::getline(file, line); // End of line
    std::getline(file, line); // Skip blank line
    std::getline(file, line); // Skip section header
    READ(env, file, length_stiffness);
    READ(env, file, length_damping);
    READ(env, file, length_motor_inertia);
    READ(env, file, length_motor_damping);
    READ(env, file, length_motor_ratio);
    READ(env, file, length_min);
    READ(env, file, length_max);

    std::getline(file, line); // End of line
    std::getline(file, line); // Skip blank line
    std::getline(file, line); // Skip section header
    READ(env, file, angle_stiffness);
    READ(env, file, angle_damping);
    READ(env, file, angle_motor_inertia);
    READ(env, file, angle_motor_damping);
    READ(env, file, angle_motor_ratio);
    READ(env, file, angle_min);
    READ(env, file, angle_max);

    std::getline(file, line); // End of line
    std::getline(file, line); // Skip blank line
    std::getline(file, line); // Skip section header
    READ(env, file, gravity);

    // Read ground vertex data
    env.ground.clear();
    std::getline(file, line); // End of line
    std::getline(file, line); // Skip header
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        GroundVertex v;
        iss >> v.x >> v.y >> v.stiffness >> v.damping >> v.friction;
        if (iss)
            env.ground.push_back(v);
        else
            break;
    }

    // End of line already skipped
    std::getline(file, line); // Skip section header
    READ(env, file, dt);
    READ(env, file, length_hardstop_kp);
    READ(env, file, length_hardstop_kd);
    READ(env, file, length_hardstop_dfade);
    READ(env, file, length_hardstop_fmax);
    READ(env, file, angle_hardstop_kp);
    READ(env, file, angle_hardstop_kd);
    READ(env, file, angle_hardstop_dfade);
    READ(env, file, angle_hardstop_fmax);
    READ(env, file, ground_damping_depth);
    READ(env, file, ground_slip_ramp);
}


void load(StateSeries& states, std::string filename)
{
    // Open file for reading
    std::ifstream file(filename);

    // Read states into vector
    states.clear();
    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        double t;
        State s;
        iss >> t;
        iss >> s.x  >> s.y  >> s.phi;
        iss >> s.dx >> s.dy >> s.dphi;
        iss >> s.leg_a.l      >> s.leg_a.l_eq;
        iss >> s.leg_a.theta  >> s.leg_a.theta_eq;
        iss >> s.leg_a.dl     >> s.leg_a.dl_eq;
        iss >> s.leg_a.dtheta >> s.leg_a.dtheta_eq;
        iss >> s.leg_b.l      >> s.leg_b.l_eq;
        iss >> s.leg_b.theta  >> s.leg_b.theta_eq;
        iss >> s.leg_b.dl     >> s.leg_b.dl_eq;
        iss >> s.leg_b.dtheta >> s.leg_b.dtheta_eq;
        if (iss)
            states.push_back({t, s});
    }
}


} // namespace sim
