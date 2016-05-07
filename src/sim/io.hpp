#pragma once

#include <string>
#include <vector>
#include <ostream>
#include "State.hpp"
#include "Environment.hpp"

namespace sim
{


void save(const Environment& env, std::string filename);
void save(const std::vector<State>& states, std::string filename);

void load(Environment& env, std::string filename);
void load(std::vector<State>& states, std::string filename);


} // namespace sim
