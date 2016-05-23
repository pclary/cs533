#include "Biped.hpp"
#include <cmath>
#include <cstddef>

#define RAD2DEG (-180.f / M_PI)

namespace vis
{


Biped::Biped(const sim::Environment& env) :
    window(sf::VideoMode(640, 480, 32),
           "Biped Visualization",
           sf::Style::Default,
           sf::ContextSettings(0, 0, 0)),
    view(sf::Vector2f(0.f, 0.f),
         sf::Vector2f(env.length_max * 4, env.length_max * 3)),
    body_radius(env.length_max / 8),
    body_fill(sf::TrianglesFan, 2),
    body_outline(sf::LinesStrip),
    leg_a(sf::Lines, 2),
    spring_a(sf::LinesStrip, 8),
    foot_a(sf::Lines, 2),
    leg_b(sf::Lines, 2),
    spring_b(sf::LinesStrip, 8),
    foot_b(sf::Lines, 2),
    grid(sf::Lines),
    ground(sf::LinesStrip)
{
    // Construct hopper
    const int num_circle_points = 64;
    body_outline.append({sf::Vector2f(body_radius / 3, 0.f), sf::Color::Black});
    body_fill.append({sf::Vector2f(0.f, 0.f), sf::Color::White});
    for (int i = 0; i < num_circle_points + 1; ++i)
    {
        const float angle = i * 2 * M_PI / num_circle_points;
        body_outline.append({body_radius *
                    sf::Vector2f(std::cos(angle), std::sin(angle)),
                    sf::Color::Black});
        body_fill.append({body_radius *
                    sf::Vector2f(std::cos(angle), std::sin(angle)),
                    sf::Color::White});
    }

    // Leg A
    leg_a[0] = {sf::Vector2f(0.f, 0.f), sf::Color::Black};
    leg_a[1] = {sf::Vector2f(0.f, 1.f), sf::Color::Black};

    spring_a[0] = {sf::Vector2f(0.f, 0.f), sf::Color::Black};
    spring_a[1] = {sf::Vector2f( body_radius, 1.f / 12), sf::Color::Black};
    spring_a[2] = {sf::Vector2f(-body_radius, 3.f / 12), sf::Color::Black};
    spring_a[3] = {sf::Vector2f( body_radius, 5.f / 12), sf::Color::Black};
    spring_a[4] = {sf::Vector2f(-body_radius, 7.f / 12), sf::Color::Black};
    spring_a[5] = {sf::Vector2f( body_radius, 9.f / 12), sf::Color::Black};
    spring_a[6] = {sf::Vector2f(-body_radius, 11.f / 12), sf::Color::Black};
    spring_a[7] = {sf::Vector2f(0.f, 1.f), sf::Color::Black};

    foot_a[0] = {sf::Vector2f(0.f, body_radius), sf::Color::Black};
    foot_a[1] = {sf::Vector2f(0.f, 0.f), sf::Color::Black};

    // Leg B
    leg_b[0] = {sf::Vector2f(0.f, 0.f), sf::Color::Black};
    leg_b[1] = {sf::Vector2f(0.f, 1.f), sf::Color::Black};

    spring_b[0] = {sf::Vector2f(0.f, 0.f), sf::Color::Black};
    spring_b[1] = {sf::Vector2f( body_radius, 1.f / 12), sf::Color::Black};
    spring_b[2] = {sf::Vector2f(-body_radius, 3.f / 12), sf::Color::Black};
    spring_b[3] = {sf::Vector2f( body_radius, 5.f / 12), sf::Color::Black};
    spring_b[4] = {sf::Vector2f(-body_radius, 7.f / 12), sf::Color::Black};
    spring_b[5] = {sf::Vector2f( body_radius, 9.f / 12), sf::Color::Black};
    spring_b[6] = {sf::Vector2f(-body_radius, 11.f / 12), sf::Color::Black};
    spring_b[7] = {sf::Vector2f(0.f, 1.f), sf::Color::Black};

    foot_b[0] = {sf::Vector2f(0.f, body_radius), sf::Color::Black};
    foot_b[1] = {sf::Vector2f(0.f, 0.f), sf::Color::Black};

    // Construct environment
    const int hgrid_div = 4;
    const int vgrid_div = 3;
    const int alpha = 64;
    for (int i = 0; i < hgrid_div + 1; ++i)
    {
        grid.append({sf::Vector2f(i, -1.f), sf::Color(255, 0, 0, alpha)});
        grid.append({sf::Vector2f(i, vgrid_div + 1),
                    sf::Color(255, 0, 0, alpha)});
    }
    for (int i = 0; i < vgrid_div + 1; ++i)
    {
        grid.append({sf::Vector2f(-1.f, i), sf::Color(255, 0, 0, alpha)});
        grid.append({sf::Vector2f(hgrid_div + 1, i),
                    sf::Color(255, 0, 0, alpha)});
    }

    for (auto gv : env.ground)
        ground.append({sf::Vector2f(gv.x, -gv.y), sf::Color::Blue});
}


void Biped::update(sim::State state)
{
    // Check if visualization is still alive (i.e. window hasn't been closed)
    if (!isAlive())
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
    view.setCenter(sf::Vector2f(state.x, -state.y + view.getSize().y / 6));

    // Set up hopper transforms
    const float spring_length = body_radius * 2;
    const float foot_length = body_radius;
    sf::Transform body_tf;
    body_tf.translate(state.x, -state.y).rotate(state.phi * RAD2DEG);

    sf::Transform leg_a_tf;
    sf::Transform spring_a_tf;
    sf::Transform foot_a_tf;
    leg_a_tf.rotate(state.leg_a.theta * RAD2DEG)
        .scale(1.f, state.leg_a.l_eq - spring_length - foot_length);
    spring_a_tf.rotate(state.leg_a.theta * RAD2DEG)
        .translate(0.f, state.leg_a.l_eq - spring_length - foot_length)
        .scale(1.f, spring_length + state.leg_a.l - state.leg_a.l_eq);
    foot_a_tf.rotate(state.leg_a.theta * RAD2DEG)
        .translate(0.f, state.leg_a.l - foot_length);

    sf::Transform leg_b_tf;
    sf::Transform spring_b_tf;
    sf::Transform foot_b_tf;
    leg_b_tf.rotate(state.leg_b.theta * RAD2DEG)
        .scale(1.f, state.leg_b.l_eq - spring_length - foot_length);
    spring_b_tf.rotate(state.leg_b.theta * RAD2DEG)
        .translate(0.f, state.leg_b.l_eq - spring_length - foot_length)
        .scale(1.f, spring_length + state.leg_b.l - state.leg_b.l_eq);
    foot_b_tf.rotate(state.leg_b.theta * RAD2DEG)
        .translate(0.f, state.leg_b.l - foot_length);

    // Grid transform
    const sf::Vector2f view_topleft = view.getCenter() - view.getSize() / 2.f;
    sf::Transform grid_tf;
    grid_tf.translate(sf::Vector2f(std::floor(view_topleft.x),
                                   std::floor(view_topleft.y)));

    // Draw stuff to window
    window.draw(grid, grid_tf);
    window.draw(foot_a, body_tf * foot_a_tf);
    window.draw(spring_a, body_tf * spring_a_tf);
    window.draw(leg_a, body_tf * leg_a_tf);
    window.draw(foot_b, body_tf * foot_b_tf);
    window.draw(spring_b, body_tf * spring_b_tf);
    window.draw(leg_b, body_tf * leg_b_tf);
    window.draw(body_fill, body_tf);
    window.draw(body_outline, body_tf);
    window.draw(ground);

    // Display visualization
    window.setView(view);
    window.display();
}


void Biped::animate(sim::StateSeries states, double rate)
{
    sf::Clock clock;
    double clock_time = 0.0;
    double t_start = states.front().time;
    double t = t_start;

    while (isAlive() && t <= states.back().time)
    {
        // Get interpolated state at time t
        size_t i;
        for (i = 0; i < states.size(); ++i)
            if (states[i].time > t) break;
        double p = (t - states[i-1].time) / (states[i].time - states[i-1].time);
        sim::State s = states[i-1].state +
            (p * (states[i].state - states[i-1].state));

        // Update display
        update(s);

        // Wait 1/60 seconds between frames to not waste effort
        while (clock.getElapsedTime().asSeconds() - clock_time < 1.0/60);

        clock_time = clock.getElapsedTime().asSeconds();
        t = t_start + (clock_time * rate);
    }

}


} // namespace vis


