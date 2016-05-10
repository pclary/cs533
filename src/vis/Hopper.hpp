#pragma once

#include <SFML/Graphics.hpp>
#include "sim/sim.hpp"


namespace vis
{


class Hopper
{
public:
    Hopper(const sim::Environment& env);
    void update(sim::State state);

private:
    sf::RenderWindow window;
    sf::View view;

    // Robot
    sf::CircleShape body;
    sf::VertexArray indicator;
    sf::VertexArray leg;
    sf::VertexArray spring;
    sf::VertexArray foot;

    // Environment
    sf::VertexArray grid;
    sf::VertexArray ground;
};


} // namespace vis
