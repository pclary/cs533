#include "Hopper.hpp"
#include <cmath>

namespace vis
{


Hopper::Hopper(const sim::Environment& env) :
    window(sf::VideoMode(640, 480, 32), "Hopper Visualization"),
    view(sf::Vector2f(0.f, 0.f),
         sf::Vector2f(env.length_max * 4, env.length_max * 3)),
    body(env.length_max / 8),
    indicator(sf::Lines, 2),
    leg(sf::Lines, 2),
    spring(sf::LinesStrip, 8),
    foot(sf::Lines, 2),
    grid(sf::Lines),
    ground(sf::LinesStrip)
{
    const float br = body.getRadius();

    // Construct hopper
    indicator[0] = sf::Vector2f(br / 3, 0.f);
    indicator[1] = sf::Vector2f(br, 0.f);

    leg[0] = sf::Vector2f(0.f, 0.f);
    leg[1] = sf::Vector2f(0.f, -1.f);

    spring[0] = sf::Vector2f(0.f, 0.f);
    spring[1] = sf::Vector2f( br, -1.f / 12);
    spring[2] = sf::Vector2f(-br, -3.f / 12);
    spring[3] = sf::Vector2f( br, -5.f / 12);
    spring[4] = sf::Vector2f(-br, -7.f / 12);
    spring[5] = sf::Vector2f( br, -9.f / 12);
    spring[6] = sf::Vector2f(-br, -11.f / 12);
    spring[7] = sf::Vector2f(0.f, -1.f);

    foot[0] = sf::Vector2f(0.f, 0.f);
    foot[1] = sf::Vector2f(0.f, -br);

    // Construct environment
    const int hgrid_div = 4;
    const int vgrid_div = 3;
    for (int i = 0; i < hgrid_div + 1; ++i)
    {
        grid.append(sf::Vector2f(float(i) / hgrid_div, -1.f));
        grid.append(sf::Vector2f(float(i) / hgrid_div,  2.f));
    }
    for (int i = 0; i < vgrid_div + 1; ++i)
    {
        grid.append(sf::Vector2f(-1.f, float(i) / vgrid_div));
        grid.append(sf::Vector2f( 2.f, float(i) / vgrid_div));
    }

    for (auto gv : env.ground)
        ground.append(sf::Vector2f(gv.x, gv.y));
}


void Hopper::update(sim::State state)
{
    // Check if window is still open
    if (!window.isOpen())
        return;

    // Respond to window events
    sf::Event event;
    while (window.pollEvent(event))
    {
        if (event.type == sf::Event::Closed)
            window.close();
    }

    // Clear window
    window.clear(sf::Color::White);

    // Set view
    view.setCenter(sf::Vector2f(state.x, state.y + view.getSize().x / 3));

    // Set up hopper transforms
    // Draw stuff to window

    // Display visualization
    window.setView(view);
    window.display();
}


} // namespace vis


