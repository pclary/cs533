#include "io.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <exception>

namespace sim
{


// Convenience macros
#define WRITE(obj, stream, name) do {                                   \
        stream << std::setw(22) << std::left;                           \
        stream << #name ": " << obj.name << std::endl;                  \
    } while (0)
#define READ(obj, stream, name) do {                                    \
        std::string label;                                              \
        stream >> label >> obj.name;                                    \
        if (label != #name ":")                                         \
            throw std::runtime_error("Bad input file format");          \
    } while (0)


void save(const Environment& env, std::string filename)
{
    // Open file for writing
    std::ofstream file(filename);

    // Write values from Environment structure
    WRITE(env, file, body_mass);
    WRITE(env, file, body_inertia);
    WRITE(env, file, foot_mass);
    WRITE(env, file, length_stiffness);
    WRITE(env, file, length_damping);
    WRITE(env, file, length_motor_inertia);
    WRITE(env, file, length_motor_damping);
    WRITE(env, file, length_motor_ratio);
    WRITE(env, file, angle_stiffness);
    WRITE(env, file, angle_damping);
    WRITE(env, file, angle_motor_inertia);
    WRITE(env, file, angle_motor_damping);
    WRITE(env, file, angle_motor_ratio);
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

    // Read values into Environment structure
    READ(env, file, body_mass);
    READ(env, file, body_inertia);
    READ(env, file, foot_mass);
    READ(env, file, length_stiffness);
    READ(env, file, length_damping);
    READ(env, file, length_motor_inertia);
    READ(env, file, length_motor_damping);
    READ(env, file, length_motor_ratio);
    READ(env, file, angle_stiffness);
    READ(env, file, angle_damping);
    READ(env, file, angle_motor_inertia);
    READ(env, file, angle_motor_damping);
    READ(env, file, angle_motor_ratio);
    READ(env, file, gravity);

    // Read ground vertex data
    env.ground.clear();
    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        GroundVertex v;
        iss >> v.x >> v.y >> v.stiffness >> v.damping >> v.friction;
        if (iss)
            env.ground.push_back(v);
    }
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
