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
    void animate(sim::StateSeries states, double rate = 1.0);

private:
    sf::RenderWindow window;
    sf::View view;

    // Robot
    const float body_radius;
    sf::VertexArray body_fill;
    sf::VertexArray body_outline;
    sf::VertexArray leg;
    sf::VertexArray spring;
    sf::VertexArray foot;

    // Environment
    sf::VertexArray grid;
    sf::VertexArray ground;
};


} // namespace vis
