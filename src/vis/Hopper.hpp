#pragma once

#include <SFML/Graphics.hpp>
#include "../sim/sim.hpp"


namespace vis
{


class Hopper
{
public:
    Hopper(const sim::Environment& env);
    void update(sim::State state);
    void animate(sim::StateSeries states, double rate = 1.0);
    bool isAlive() { return window.isOpen(); }

private:
    sf::RenderWindow window;
    sf::View view;

    // Robot
    const float body_radius;
    sf::VertexArray body_fill;
    sf::VertexArray body_outline;
    sf::VertexArray leg_a;
    sf::VertexArray spring_a;
    sf::VertexArray foot_a;
    sf::VertexArray leg_b;
    sf::VertexArray spring_b;
    sf::VertexArray foot_b;

    // Environment
    sf::VertexArray grid;
    sf::VertexArray ground;
};


} // namespace vis
