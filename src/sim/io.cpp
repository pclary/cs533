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


void save(const std::vector<State>& states, std::string filename)
{
    // Open file for writing
    std::ofstream file(filename);

    // Write states from vector
    file << "states: ";
    file << "[x y phi theta theta_eq l l_eq (derivatives...)]" << std::endl;
    for (const auto s : states)
    {
        file << s.x         << " ";
        file << s.y         << " ";
        file << s.phi       << " ";
        file << s.l         << " ";
        file << s.l_eq      << " ";
        file << s.theta     << " ";
        file << s.theta_eq  << " ";
        file << s.dx        << " ";
        file << s.dy        << " ";
        file << s.dphi      << " ";
        file << s.dl        << " ";
        file << s.dl_eq     << " ";
        file << s.dtheta    << " ";
        file << s.dtheta_eq << std::endl;
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


void load(std::vector<State>& states, std::string filename)
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
        State s;
        iss >> s.x  >> s.y     >> s.phi;
        iss >> s.l  >> s.l_eq  >> s.theta  >> s.theta_eq;
        iss >> s.dx >> s.dy    >> s.dphi;
        iss >> s.dl >> s.dl_eq >> s.dtheta >> s.dtheta_eq;
        if (iss)
            states.push_back(s);
    }
}


} // namespace sim
