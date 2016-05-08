#include "dynamics.hpp"
#include <array>
#include <cmath>
#include <vector>

namespace sim
{


/*******************************************************************************
 * Private functions
 ******************************************************************************/

namespace
{


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


inline MotorTorques operator+ (const MotorTorques& a, const MotorTorques& b)
{
    return {a.length + b.length, a.angle + b.angle};
}


inline MotorTorques low_level_controller(State state, double t,
                                         ControllerTarget target,
                                         ControllerParams params)
{
    return {0, 0};
}


inline DState hopper_eom(State state, const Environment& env,
                  MotorTorques motors, ExternalForces ext)
{
    // Calculate motor gap torques, taking damping into account
    const double angle_motor_gap_torque = motors.angle -
        (env.angle_motor_damping * state.dtheta_eq * env.angle_motor_ratio);
    const double length_motor_gap_torque = motors.length -
        (env.length_motor_damping * state.dl_eq * env.length_motor_ratio);

    // Calculate internal spring forces
    const double length_compression = state.l_eq - state.l;
    const double length_force = env.length_stiffness * length_compression;
    const double angle_compression = state.theta_eq - state.theta;
    const double angle_torque = env.angle_stiffness * angle_compression;

    // Get basis vectors for internal spring forces
    // Positive when acting on the foot, negate for body
    const double l_x = std::sin(state.theta + state.phi);
    const double l_y = -std::cos(state.theta + state.phi);
    const double theta_x = -l_y;
    const double theta_y = l_x;

    // Forces on body
    const double force_body_x = ext.body_x - (l_x * length_force) -
        (theta_x * angle_torque * state.l);
    const double force_body_y = ext.body_y - (l_y * length_force) -
        (theta_y * angle_torque * state.l);
    const double torque_body_phi = ext.body_phi - angle_motor_gap_torque -
        (1.0 - 1.0 / env.angle_motor_ratio) * angle_torque;

    // Acceleration of body
    const double ddx = force_body_x / env.body_mass;
    const double ddy = force_body_y / env.body_mass;
    const double ddphi = torque_body_phi / env.body_inertia;

    // Acceleration of leg equilibrium positions
    const double ddtheta_eq = (angle_motor_gap_torque -
                               angle_torque / env.angle_motor_ratio) /
        (env.angle_motor_ratio * env.angle_motor_inertia);
    const double ddl_eq = (length_motor_gap_torque -
                           length_force / env.length_motor_ratio) /
        (env.length_motor_ratio * env.length_motor_inertia);

    // Convert external forces on foot to relative polar coordinate acceleration
    // Gravity is included in the external forces
    const double accel_offset_foot_x = ext.foot_x / env.foot_mass - ddx;
    const double accel_offset_foot_y = ext.foot_y / env.foot_mass - ddy;
    const double accel_foot_l = (accel_offset_foot_x * l_x) +
        (accel_offset_foot_y * l_y);
    const double accel_foot_theta = (accel_offset_foot_x * theta_x) +
        (accel_offset_foot_y * theta_y) - ddphi;

    // Acceleration of actual leg positions
    const double dtheta_abs = state.dtheta + state.dphi;
    const double ddl = accel_foot_l + (state.l * dtheta_abs * dtheta_abs);
    const double ddtheta = (accel_foot_theta -
                            (2 * state.dl * dtheta_abs)) / state.l;

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
    return clamp(std::fabs(x_over / fade), lower, upper);
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


inline DState hopper_dynamics(State state, double t, const Environment& env,
                       ControllerTarget target, ControllerParams params)
{
    // Get motor torques from low-level controller
    const MotorTorques motors = low_level_controller(state, t, target, params);

    // Get hardstop forces
    const MotorTorques hardstops = hardstop_forces(state, env);

    // Calculate external forces
    ExternalForces ext = {};
    // Needs environment interaction
    ext.body_y -= env.body_mass * env.gravity;
    ext.foot_y -= env.foot_mass * env.gravity;

    return hopper_eom(state, env, motors + hardstops, ext);
}


inline TimeState integration_step(TimeState ts, double dt, const Environment& env,
                           ControllerTarget target, ControllerParams params)
{
    // Performs a 4th order runge-kutta integration step
    // dt is passed explicitly instead of using env.dt so that the
    // integrator can take a short final timestep
    const State   s0 = ts.state;
    const DState ds0 = hopper_dynamics(s0, ts.time, env, target, params);
    const State   s1 = s0 + ds0 * (dt/2);
    const DState ds1 = hopper_dynamics(s1, ts.time + dt/2, env, target, params);
    const State   s2 = s0 + ds1 * (dt/2);
    const DState ds2 = hopper_dynamics(s2, ts.time + dt/2, env, target, params);
    const State   s3 = s0 + ds2 * dt;
    const DState ds3 = hopper_dynamics(s3, ts.time + dt, env, target, params);
    return {ts.time + dt, s0 + (ds0 + 2*ds1 + 2*ds2 + ds3) * (dt/6)};
}


} // namespace


/*******************************************************************************
 * Public functions
 ******************************************************************************/

StateSeries simulate_hopper(State initial, double stop_time, Environment env,
                            ControllerTarget target, ControllerParams params)
{
    // Initialize output vector
    StateSeries output = {{0.0, initial}};
    output.reserve(stop_time / env.dt);

    // Step simulation forward until next timestep will put it over stop time
    while (output.back().time + env.dt < stop_time)
        output.push_back(integration_step(output.back(), env.dt,
                                          env, target, params));

    // Take a short final step to hit the desired stop time
    const double dt_final = stop_time - output.back().time;
    output.push_back(integration_step(output.back(), dt_final,
                                      env, target, params));

    return output;
}


} // namespace sim
