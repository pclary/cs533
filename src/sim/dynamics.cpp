#include "dynamics.hpp"
#include <array>
#include <cmath>
#include <vector>
#include <limits>
#include <cstddef>
#include <utility>

namespace sim
{

/*******************************************************************************
 * Private functions and data structures
 ******************************************************************************/

namespace
{

/*******************************************************************************
 * Data structures
 ******************************************************************************/

struct ExternalForces
{
    double body_x;
    double body_y;
    double body_phi;
    double foot_x;
    double foot_y;
};


struct MotorTorques
{
    double length;
    double angle;
};


struct Vec2
{
    double x;
    double y;
};


inline MotorTorques operator+ (const MotorTorques& a, const MotorTorques& b)
{
    return {a.length + b.length, a.angle + b.angle};
}


/*******************************************************************************
 * Miscellaneous functions
 ******************************************************************************/

inline double clamp(double x, double lower, double upper)
{
    // Clamp x to the lower and upper bounds
    return std::fmin(std::fmax(x, lower), upper);
}


inline double fade_derivative(double x, double lower, double upper, double fade)
{
    // Returns 1 when outside of lower and upper bounds, fades to 0
    // along distance fade within bounds
    const double x_over = x - clamp(x, lower + fade, upper - fade);
    return clamp(std::fabs(x_over / fade), 0, 1);
}


inline std::pair<GroundData, Vec2> nearest_ground_point(double x, double y,
                                                        const Environment& env)
{
    // Find the point on the ground closest to the point to test
    double min_dist2 = std::numeric_limits<double>::max();
    double min_p = 0.0;
    double min_x_line = 0.0;
    double min_y_line = 0.0;
    size_t min_index = 0;
    for (size_t i = 0; i < env.ground.size() - 1; ++i)
    {
        const double xg = env.ground[i].x;
        const double yg = env.ground[i].y;
        const double dxg = env.ground[i + 1].x - xg;
        const double dyg = env.ground[i + 1].y - yg;

        // Take dot product to project test point onto line, then
        // normalize with the segment length squared and clamp to keep
        // within line segment bounds
        const double dot_product = (x - xg) * dxg + (y - yg) * dyg;
        const double seg_length2 = (dxg * dxg) + (dyg * dyg);
        const double p = clamp(dot_product / seg_length2, 0, 1);

        // Nearest point on the line segment to the test point
        const double x_line = xg + (p * dxg);
        const double y_line = yg + (p * dyg);

        // Squared distance from line point to test point
        const double dist2 = ((x - x_line) * (x - x_line)) +
            ((y - y_line) * (y - y_line));

        // If this is a new minimum, save values
        // Ignore segments with zero length
        if (dist2 < min_dist2 && seg_length2 > 0.0)
        {
            min_dist2 = dist2;
            min_p = p;
            min_x_line = x_line;
            min_y_line = y_line;
            min_index = i;
        }
    }

    // Check whether point is on the ground side (right hand side) of the line
    const double dxg = env.ground[min_index + 1].x - env.ground[min_index].x;
    const double dyg = env.ground[min_index + 1].y - env.ground[min_index].y;
    const double dxp = point.x - env.ground[min_index].x;
    const double dyp = point.y - env.ground[min_index].y;
    const double cross_product = (dxg * dyp) - (dyg * dxp);
    const bool in_ground = cross_product <= 0.0;

    // Calculate ground normal
    const double dist = std::sqrt(min_dist2);
    const double normal_x = (in_ground ? -1.0 * 1.0) * (x - min_x_line) / dist;
    const double normal_y = (in_ground ? -1.0 * 1.0) * (y - min_y_line) / dist;

    // Get interpolated ground properties
    const double ground_stiffness = env.ground[min_index].stiffness +
        (min_p * (env.ground[min_index + 1].stiffness -
                  env.ground[min_index].stiffness));
    const double ground_damping = env.ground[min_index].damping +
        (min_p * (env.ground[min_index + 1].damping -
                  env.ground[min_index].damping));
    const double ground_friction = env.ground[min_index].friction +
        (min_p * (env.ground[min_index + 1].friction -
                  env.ground[min_index].friction));

    // Return the data for the nearest ground point to the test point
    // and whether the test point in inside the ground
    return {{min_x_line,
             min_y_line,
             ground_stiffness,
             ground_damping,
             ground_friction},
            {normal_x, normal_y}};
}


/*******************************************************************************
 * Low-level controller
 ******************************************************************************/

struct ControllerState
{

};

inline MotorTorques low_level_controller(State state,
                                         double t,
                                         const Environment& env,
                                         ControllerTarget target,
                                         ControllerParams params,
                                         ControllerState& cstate)
{
    return {0, 0};
}


/*******************************************************************************
 * Equations of motion
 ******************************************************************************/

inline DState hopper_eom(State state,
                         const Environment& env,
                         MotorTorques motors,
                         ExternalForces ext)
{
    // Calculate motor gap torques, taking damping into account
    const double angle_motor_gap_torque = motors.angle -
        (env.angle_motor_damping * state.dtheta_eq * env.angle_motor_ratio);
    const double length_motor_gap_torque = motors.length -
        (env.length_motor_damping * state.dl_eq * env.length_motor_ratio);

    // Calculate internal spring forces
    const double length_spring_force =
        (env.length_stiffness * (state.l_eq - state.l)) +
        (env.length_damping * (state.dl_eq - state.dl));
    const double angle_spring_torque =
        (env.angle_stiffness * (state.theta_eq - state.theta)) +
        (env.angle_damping * (state.dtheta_eq - state.dtheta));
    const double angle_spring_force = angle_spring_torque / state.l;

    // Get basis vectors for internal spring forces
    // Positive when acting on the foot, negate for body
    const double l_x = std::sin(state.theta + state.phi);
    const double l_y = -std::cos(state.theta + state.phi);
    const double theta_x = -l_y;
    const double theta_y = l_x;

    // Forces on body
    const double force_body_x = ext.body_x - (l_x * length_spring_force) -
        (theta_x * angle_spring_force);
    const double force_body_y = ext.body_y - (l_y * length_spring_force) -
        (theta_y * angle_spring_force);
    const double torque_body_phi = ext.body_phi - angle_motor_gap_torque -
        (1.0 - 1.0 / env.angle_motor_ratio) * angle_spring_torque;

    // Acceleration of body
    const double ddx = force_body_x / env.body_mass;
    const double ddy = force_body_y / env.body_mass;
    const double ddphi = torque_body_phi / env.body_inertia;

    // Acceleration of leg equilibrium positions
    const double ddtheta_eq = (angle_motor_gap_torque -
                               angle_spring_torque / env.angle_motor_ratio) /
        (env.angle_motor_ratio * env.angle_motor_inertia);
    const double ddl_eq = (length_motor_gap_torque -
                           length_spring_force / env.length_motor_ratio) /
        (env.length_motor_ratio * env.length_motor_inertia);

    // Convert external forces on foot to relative polar coordinate acceleration
    // Gravity is included in the external forces
    const double accel_offset_foot_x = ext.foot_x / env.foot_mass - ddx;
    const double accel_offset_foot_y = ext.foot_y / env.foot_mass - ddy;
    const double accel_foot_l = (length_spring_force / env.foot_mass) +
        (accel_offset_foot_x * l_x) + (accel_offset_foot_y * l_y);
    const double accel_foot_theta = (angle_spring_force / env.foot_mass) +
        (accel_offset_foot_x * theta_x) + (accel_offset_foot_y * theta_y);

    // Acceleration of actual leg positions
    const double dtheta_abs = state.dtheta + state.dphi;
    const double ddl = accel_foot_l + (state.l * dtheta_abs * dtheta_abs);
    const double ddtheta = (accel_foot_theta -
                            (2 * state.dl * dtheta_abs)) / state.l - ddphi;

    // Output state derivative vector
    return {state.dx,
            state.dy,
            state.dphi,
            state.dl,
            state.dl_eq,
            state.dtheta,
            state.dtheta_eq,
            ddx,
            ddy,
            ddphi,
            ddl,
            ddl_eq,
            ddtheta,
            ddtheta_eq};
}


inline void aerial_foot_phase(double& h, double& dh, double ddh,
                              double& p, double& dp, double ddp,
                              double& t, double dt)
{
    // Use quadratic equation to find first upcoming zero crossing
    const double discriminant = (dh * dh) - (2 * ddh * h);

    if (discriminant < 0 || (dh > 0 && ddh > 0))
    {
       // No upcoming zero crossings
        const double t_diff = dt - t;
        t = dt;
        h += (dh * t_diff) + (0.5 * ddh * t_diff * t_diff);
        p += (dp * t_diff) + (0.5 * ddp * t_diff * t_diff);
    }

    // Calculate the time of the first zero crossing
    // Different quadratic solution depending on whether the
    // trajectory is concave up or down
    const double t_zc =
        (-dh + (ddh > 0 ? -1.0 : 1.0) * std::sqrt(discriminant)) / ddh;

    // Check whether this completes the timestep and add the
    // appropriate values to the foot position and time
    if (t + t_zc >= dt)
    {
        const double t_diff = dt - t;
        t = dt;
        h += (dh * t_diff) + (0.5 * ddh * t_diff * t_diff);
        p += (dp * t_diff) + (0.5 * ddp * t_diff * t_diff);
    }
    else
    {
        t += t_zc;
        h += (dh * t_zc) + (0.5 * ddh * t_zc * t_zc);
        p += (dp * t_zc) + (0.5 * ddp * t_zc * t_zc);
    }
}


inline void ground_foot_phase(double& h, double& dh, double ddh,
                              double& p, double& dp, double ddp,
                              double& t, double dt,
                              GroundData gd, const Environment& env)
{
    // Foot is assumed to be in the ground at this point
    // h == 0.0 counts as in ground

    // Check whether first positive peak is above ground to determine
    // whether a new flight phase is required
    // Use undamped oscillations to approximate peaks -- not quite
    // correct but close in all reasonable circumstances, and works
    // for all damping cases
    const double wn = std::sqrt(gd.stiffness / env.foot_mass);
    const double zeta = gd.damping/2 / std::sqrt(gd.stiffness * env.foot_mass);
    const double phase = -std::atan2(h, ((wn * zeta + dh) / wn);
    const double t_peak = std::modf(phase + M_PI/2, 2*M_PI) / wn;
    const double h_offset = (ddh * env.foot_mass) / gd.stiffness;

    // Use a few iterations of Newton's method to find the zero crossing
    if (zeta < 0.99)
    {
        // Underdamped
        const double wd = wn * std::sqrt(1.0 - zeta * zeta);
        const double l = wn * zeta;

    }
    else if (zeta > 1.01)
    {
        // Overdamped
    }
    else
    {
        // Use the critically damped solution when zeta is close to 1
        // to prevent numerical issues
    }

    // Compute the new tangential velocity and position using friction
    // and acceleration impulse
    // Ground normal force for friction is estimated by adding the
    // normal spring force and the impulse from the dh calculation


}


inline State foot_ground_contact(State state_old,
                                 State state_new,
                                 double dt,
                                 const Environment& env)
{
    // If the foot is in the ground, use piecewise analytic solutions
    // to find its new state

    // Get ground properties at the new location
    const double l_x_new = std::sin(state_new.theta + state_new.phi);
    const double l_y_new = -std::cos(state_new.theta + state_new.phi);
    const double foot_x_new = state_new.x + (l_x_new * state_new.l);
    const double foot_y_new = state_new.y + (l_y_new * state_new.l);
    const auto ret = nearest_ground_point(foot_x_new, foot_y_new, env);
    const GroundData gd = ret.first;
    const Vec2 normal = ret.second;

    // Compute height above ground for old and new state
    const double l_x_old = std::sin(state_old.theta + state_old.phi);
    const double l_y_old = -std::cos(state_old.theta + state_old.phi);
    const double foot_x_old = state_old.x + (l_x_old * state_old.l);
    const double foot_y_old = state_old.y + (l_y_old * state_old.l);
    const double h_old = (normal.x * (foot_x_old - gd.x)) +
        (normal.y * (foot_y_old - gd.y));
    const double h_new = (normal.x * (foot_x_new - gd.x)) +
        (normal.y * (foot_y_new - gd.y));

    // If new and old state are both above the ground, return the
    // state computed by the normal integration without modifying it
    if (h_old > 0.0 && h_new > 0.0)
        return state_new;

    // Otherwise, alternate between above and in-ground phases
    // starting from the old state until time dt is elapsed
    double t = 0.0;

    // Compute initial state in ground normal/tangent coordinates (h/p)
    const double theta_x_old = -l_y_old;
    const double theta_y_old = l_x_old;
    const double foot_dx_old = state_old.dx + (l_x_old * state_old.dl) +
        (theta_x_old * state_old.l * state_old.dtheta);
    const double foot_dy_old = state_old.dy + (l_y_old * state_old.dl) +
        (theta_y_old * state_old.l * state_old.dtheta);
    const Vec2 tangent = {normal.y, -normal.x};

    double h = h_old;
    double dh = (normal.x * foot_dx_old) + (normal.y * foot_dy_old);
    double p = (tangent.x * (foot_x_old - gd.x)) +
        (tangent.y * (foot_y_old - gd.y));
    double dp = (tangent.x * foot_dx_old) + (tangent.y * foot_dy_old);

    // Use old state to calculate spring forces and constant accelerations
    const double length_spring_force_old =
        (env.length_stiffness * (state_old.l_eq - state_old.l)) +
        (env.length_damping * (state_old.dl_eq - state_old.dl));
    const double angle_spring_torque_old =
        (env.angle_stiffness * (state_old.theta_eq - state_old.theta)) +
        (env.angle_damping * (state_old.dtheta_eq - state_old.dtheta));
    const double angle_spring_force_old = angle_spring_torque_old / state_old.l;
    const double spring_force_x = (l_x_old * length_spring_force_old) +
        (theta_x_old * angle_spring_force_old);
    const double spring_force_y = (l_y_old * length_spring_force_old) +
        (theta_y_old * angle_spring_force_old);

    const double ddh =
        (normal.x * (spring_force_x / env.foot_mass)) +
        (normal.y * ((spring_force_y / env) - env.gravity));
    const double ddp =
        (tangent.x * (spring_force_x / env.foot_mass)) +
        (tangent.y * ((spring_force_y / env) - env.gravity));

    // If starting above the ground, continue until contact is made
    if (h_old > 0.0)
        aerial_foot_phase(h, dh, ddh, p, dp, ddp, t, dt);

    // Alternate between in- and out-of-ground phases until the
    // timestep duration is reached
    while (t < dt)
    {
        ground_foot_phase(h, dh, ddh, p, dp, ddp, t, dt, gd);
        if (t >= dt)
            break;
        aerial_foot_phase(h, dh, ddh, p, dp, ddp, t, dt);
    }
}


inline MotorTorques hardstop_forces(State state, const Environment& env)
{
    // Compute how much each DOF is over/under the hardstops
    const double l_eq_over = state.l_eq -
        clamp(state.l_eq, env.length_min, env.length_max);
    const double theta_eq_over = state.theta_eq -
        clamp(state.theta_eq, env.angle_min, env.angle_max);

    // Fade the derivative term in near the hardstops for smoother dynamics
    const double l_eq_dfade = fade_derivative(state.l_eq,
                                              env.length_min,
                                              env.length_max,
                                              env.length_hardstop_dfade);
    const double theta_eq_dfade = fade_derivative(state.theta_eq,
                                                  env.angle_min,
                                                  env.angle_max,
                                                  env.angle_hardstop_dfade);

    // Compute hardstop forces as a spring+damper system
    const double l_eq_force = -(env.length_hardstop_kp * l_eq_over) -
        (env.length_hardstop_kd * l_eq_dfade * state.dl_eq);
    const double theta_eq_torque = -(env.angle_hardstop_kp * theta_eq_over) -
        (env.angle_hardstop_kd * theta_eq_dfade * state.dtheta_eq);

    // Clamp forces before returning them
    return {clamp(l_eq_force,
                  -env.length_hardstop_fmax,
                  env.length_hardstop_fmax),
            clamp(theta_eq_torque,
                  -env.angle_hardstop_fmax,
                  env.angle_hardstop_fmax)};
}


inline DState hopper_dynamics(State state,
                              double t,
                              const Environment& env,
                              ControllerTarget target,
                              ControllerParams params,
                              ControllerState& cstate)
{
    // Get motor torques from low-level controller
    const MotorTorques motors = low_level_controller(state, t, env, target,
                                                     params, cstate);

    // Get hardstop forces
    const MotorTorques hardstops = hardstop_forces(state, env);

    // Calculate external forces (currently just gravity, ground
    // contact is done outside the normal ODE)
    const ExternalForces ext = {
        0.0,
        -(env.body_mass * env.gravity),
        0.0,
        0.0,
        -(env.foot_mass * env.gravity)};

    return hopper_eom(state, env, motors + hardstops, ext);
}


/*******************************************************************************
 * Integrator
 ******************************************************************************/

inline TimeState integration_step(TimeState ts,
                                  double dt,
                                  const Environment& env,
                                  ControllerTarget target,
                                  ControllerParams params,
                                  ControllerState& cstate)
{
    // Performs a 4th order runge-kutta integration step
    // dt is passed explicitly instead of using env.dt so that the
    // integrator can take a short final timestep
    const State   s0 = ts.state;
    const DState ds0 = hopper_dynamics(s0, ts.time, env,
                                       target, params, cstate);
    const State   s1 = s0 + ds0 * (dt/2);
    const DState ds1 = hopper_dynamics(s1, ts.time + dt/2, env,
                                       target, params, cstate);
    const State   s2 = s0 + ds1 * (dt/2);
    const DState ds2 = hopper_dynamics(s2, ts.time + dt/2, env,
                                       target, params, cstate);
    const State   s3 = s0 + ds2 * dt;
    const DState ds3 = hopper_dynamics(s3, ts.time + dt, env,
                                       target, params, cstate);
    return {ts.time + dt, s0 + (ds0 + 2*ds1 + 2*ds2 + ds3) * (dt/6)};
}


} // namespace


/*******************************************************************************
 * Public functions
 ******************************************************************************/

StateSeries simulate_hopper(State initial,
                            double stop_time,
                            Environment env,
                            ControllerTarget target,
                            ControllerParams params)
{
    // Initialize output vector
    StateSeries output = {{0.0, initial}};
    output.reserve(stop_time / env.dt);

    // Initialize low-level controller state
    ControllerState cstate;

    // Step simulation forward until next timestep will put it over stop time
    while (output.back().time + env.dt < stop_time)
        output.push_back(integration_step(output.back(), env.dt,
                                          env, target, params, cstate));

    // Take a short final step to hit the desired stop time
    const double dt_final = stop_time - output.back().time;
    output.push_back(integration_step(output.back(), dt_final,
                                      env, target, params, cstate));

    return output;
}


} // namespace sim
