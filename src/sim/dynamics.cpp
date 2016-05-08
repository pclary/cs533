#include "dynamics.hpp"
#include <array>
#include "controller.hpp"
#include <cmath>

namespace sim
{


struct ExternalForces
{
    double body_x;
    double body_y;
    double body_phi;
    double foot_x;
    double foot_y;
};


DState hopper_eom(State state, Environment env,
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
        (1.0 - env.angle_motor_ratio) * angle_torque;

    // Acceleration of body
    const double ddx = force_body_x / env.body_mass;
    const double ddy = force_body_y / env.body_mass;
    const double ddphi = torque_body_phi / env.body_inertia;

    // Acceleration of leg equilibrium positions
    const double ddtheta_eq = (angle_motor_gap_torque -
                               angle_torque * env.angle_motor_ratio) *
        env.angle_motor_ratio / env.angle_motor_inertia;
    const double ddl_eq = (length_motor_gap_torque -
                           length_force * env.length_motor_ratio) *
        env.length_motor_ratio / env.length_motor_inertia;

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


DState hopper_dynamics(State state, double t, Environment env,
                       ControllerTarget target, ControllerParams params)
{
    // Get motor torques from low-level controller
    const MotorTorques motors = low_level_controller(state, t, target, params);

    // Calculate external forces
    ExternalForces ext = {};
    // Needs environment interaction
    ext.body_y -= env.body_mass * env.gravity;
    ext.foot_y -= env.foot_mass * env.gravity;

    return hopper_eom(state, env, motors, ext);
}


State integration_step(State s0, double t, double dt, Environment env,
                       ControllerTarget target, ControllerParams params)
{
    // Performs a 4th order runge-kutta integration step
    DState ds0 = hopper_dynamics(s0, t, env, target, params);
    State   s1 = s0 + ds0 * (dt/2);
    DState ds1 = hopper_dynamics(s1, t + dt/2, env, target, params);
    State   s2 = s0 + ds1 * (dt/2);
    DState ds2 = hopper_dynamics(s2, t + dt/2, env, target, params);
    State   s3 = s0 + ds2 * dt;
    DState ds3 = hopper_dynamics(s3, t + dt, env, target, params);
    return s0 + (ds0 + 2*ds1 + 2*ds2 + ds3) * (dt/6);
}


State simulate_hopper(State initial, Environment env, double time)
{
    // TODO
    return {};
}


} // namespace sim
