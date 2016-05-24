#include "dynamics.hpp"
#include <array>
#include <cmath>
#include <vector>
#include <limits>
#include <cstddef>

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

struct Force
{
    double x;
    double y;
};


struct ExternalForces
{
    Force  body;
    double body_phi;
    Force  foot_a;
    Force  foot_b;
};


struct LegMotorTorques
{
    double length;
    double angle;
};


struct MotorTorques
{
    LegMotorTorques leg_a;
    LegMotorTorques leg_b;
};


struct PointState
{
    double x;
    double y;
    double dx;
    double dy;
};


struct TrajectoryPoint
{
    double phase;
    double torque;
    double target;
    double kp;
    double kd;
};


inline Force operator+ (const Force& a, const Force& b)
{
    return {a.x + b.x, a.y + b.y};
}


inline LegMotorTorques operator+ (const LegMotorTorques& a,
                                  const LegMotorTorques& b)
{
    return {a.length + b.length, a.angle + b.angle};
}


inline MotorTorques operator+ (const MotorTorques& a, const MotorTorques& b)
{
    return {a.leg_a + b.leg_a, a.leg_b + b.leg_b};
}


/*******************************************************************************
 * Utility functions
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


inline TrajectoryValues interp_trajectory(const TrajectoryPoint t[],
                                          double phase)
{
    // Relies on the phase never being equal to or greater than the
    // phase of the last point in the trajectory; this is true if
    // phase is limited to [0, 1) and t has points defined at 0 and 1

    // Find the first point with a larger phase than phase
    size_t i = 0;
    while (t[++i].phase < phase);

    // Find the proportion used for linear interpolation
    double phase_diff = t[i].phase - t[i - 1].phase;
    double p = (phase - t[i - 1].phase) / phase_diff;

    // Output values
    return {phase,
            t[i - 1].torque + p * (t[i].torque - t[i - 1].torque),
            (t[i].torque - t[i - 1].torque) / phase_diff,
            t[i - 1].target + p * (t[i].target - t[i - 1].target),
            (t[i].target - t[i - 1].target) / phase_diff,
            t[i - 1].kp + p * (t[i].kp - t[i - 1].kp),
            t[i - 1].kd + p * (t[i].kd - t[i - 1].kd)};
}


inline double eval_trajectory(TrajectoryValues tvals, double x, double dx)
{
    return tvals.torque +
        (tvals.kp * (tvals.target - x)) +
        (tvals.kd * (tvals.dtarget - dx));
}


/*******************************************************************************
 * Low-level controller
 ******************************************************************************/

const TrajectoryPoint length_trajectory[] =
{
    {0.0, 6.0, 0.8, 3e1, 3e2},
    {0.2, 6.0, 0.8, 3e1, 3e2},
    {0.4, 0.0, 0.7, 1e0, 1e2},
    {0.6, 0.0, 0.7, 1e0, 1e2},
    {0.8, 6.0, 0.8, 3e1, 3e2},
    {1.0, 6.0, 0.8, 3e1, 3e2}
};


const TrajectoryPoint angle_trajectory[] =
{
    {0.0, 1.0,  0.0, 0.0, 0.0},
    {0.4, 0.0, -1.0, 4e3, 4e2},
    {0.6, 0.0,  1.0, 4e3, 4e2},
    {1.0, 1.0,  0.0, 0.0, 0.0}
};


const TrajectoryPoint body_angle_trajectory[] =
{
    {0.0, 0.0, 0.0, -1e4, -1e3},
    {0.4, 0.0, 0.0,  0.0,  0.0},
    {0.6, 0.0, 0.0,  0.0,  0.0},
    {1.0, 0.0, 0.0, -1e4, -1e3}
};


inline void low_level_controller_update(State state,
                                        double,
                                        double dt,
                                        const Environment&,
                                        ControllerTarget,
                                        ControllerParams,
                                        ControllerState& cstate)
{
    const double phase_rate = 1.6;
    cstate.phase_a += phase_rate * dt;
    cstate.phase_b += phase_rate * dt;

    // End this step when either leg's phase reaches 1
    cstate.end_step = cstate.phase_a >= 1.0 || cstate.phase_b >= 1.0;

    // Keep phases within 0-1 range
    cstate.phase_a -= floor(cstate.phase_a);
    cstate.phase_b -= floor(cstate.phase_b);

    // Get interpolated motor trajectory values
    cstate.length_tvals_a =
        interp_trajectory(length_trajectory, cstate.phase_a);
    cstate.length_tvals_b =
        interp_trajectory(length_trajectory, cstate.phase_b);

    cstate.angle_tvals_a = interp_trajectory(angle_trajectory, cstate.phase_a);
    cstate.angle_tvals_b = interp_trajectory(angle_trajectory, cstate.phase_b);

    cstate.body_angle_tvals_a =
        interp_trajectory(body_angle_trajectory, cstate.phase_a);
    cstate.body_angle_tvals_b =
        interp_trajectory(body_angle_trajectory, cstate.phase_b);

    // Record velocity at midstance
    if (cstate.end_step)
        cstate.dx_midstance = state.dx;
}


inline MotorTorques low_level_controller_output(State state,
                                                double,
                                                const Environment& env,
                                                ControllerTarget target,
                                                ControllerParams params,
                                                const ControllerState& cstate)
{
    // Evaluate length trajectory controllers
    const double length_torque_a = eval_trajectory(cstate.length_tvals_a,
                                                   state.leg_a.l_eq,
                                                   state.leg_a.dl_eq);
    const double length_torque_b = eval_trajectory(cstate.length_tvals_b,
                                                   state.leg_b.l_eq,
                                                   state.leg_b.dl_eq);

    // Angle trajectory values are modulated by velocity control and
    // horizontal push
    const double velocity_control_angle =
        0.6*cstate.dx_midstance - 0.2*target.velocity;

    TrajectoryValues angle_tvals_a = cstate.angle_tvals_a;
    angle_tvals_a.target  *= velocity_control_angle;
    angle_tvals_a.dtarget *= velocity_control_angle;
    angle_tvals_a.torque  *= params.horizontal_push;
    angle_tvals_a.dtorque *= params.horizontal_push;

    TrajectoryValues angle_tvals_b = cstate.angle_tvals_b;
    angle_tvals_b.target  *= velocity_control_angle;
    angle_tvals_b.dtarget *= velocity_control_angle;
    angle_tvals_b.torque  *= params.horizontal_push;
    angle_tvals_b.dtorque *= params.horizontal_push;

    // Body angle controller only acts when foot is on ground
    const double body_angle_torque_a =
        eval_trajectory(cstate.body_angle_tvals_a, state.phi, state.dphi);
    const double body_angle_torque_b =
        eval_trajectory(cstate.body_angle_tvals_b, state.phi, state.dphi);

    //  // Limit body angle controller torques to prevent slip
    // const double l_force_a =
    //     env.length_stiffness * (state.leg_a.l_eq - state.leg_a.l);
    // const double friction_limit_a =
    //     0.5 * std::fmax(l_force_a, 0) / state.leg_a.l / env.angle_motor_ratio;
    // const double body_angle_torque_limited_a =
    //     clamp(body_angle_torque_a, -friction_limit_a, friction_limit_a);
    // const double l_force_b =
    //     env.length_stiffness * (state.leg_b.l_eq - state.leg_b.l);
    // const double friction_limit_b =
    //     0.5 * std::fmax(l_force_b, 0) / state.leg_b.l / env.angle_motor_ratio;
    // const double body_angle_torque_limited_b =
    //     clamp(body_angle_torque_b, -friction_limit_b, friction_limit_b);

    // Angle torque is the sum of the leg and body angle controllers
    const double theta_eq_a  = state.leg_a.theta_eq + state.phi;
    const double dtheta_eq_a = state.leg_a.dtheta_eq + state.dphi;
    double angle_torque_a = body_angle_torque_a +
        eval_trajectory(angle_tvals_a, theta_eq_a, dtheta_eq_a);
    const double theta_eq_b  = state.leg_b.theta_eq + state.phi;
    const double dtheta_eq_b = state.leg_b.dtheta_eq + state.dphi;
    double angle_torque_b = body_angle_torque_b +
        eval_trajectory(angle_tvals_b, theta_eq_b, dtheta_eq_b);

    // Limit torques to motor peak output
    return {{clamp(length_torque_a, -12.2, 12.2),
             clamp(angle_torque_a,  -12.2, 12.2)},
            {clamp(length_torque_b, -12.2, 12.2),
             clamp(angle_torque_b,  -12.2, 12.2)}};
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
    const double angle_motor_gap_torque_a = motors.leg_a.angle -
        (env.angle_motor_damping * state.leg_a.dtheta_eq *
         env.angle_motor_ratio);
    const double length_motor_gap_torque_a = motors.leg_a.length -
        (env.length_motor_damping * state.leg_a.dl_eq * env.length_motor_ratio);
    const double angle_motor_gap_torque_b = motors.leg_b.angle -
        (env.angle_motor_damping * state.leg_b.dtheta_eq *
         env.angle_motor_ratio);
    const double length_motor_gap_torque_b = motors.leg_b.length -
        (env.length_motor_damping * state.leg_b.dl_eq * env.length_motor_ratio);

    // Calculate internal spring forces
    const double length_spring_force_a =
        (env.length_stiffness * (state.leg_a.l_eq - state.leg_a.l)) +
        (env.length_damping * (state.leg_a.dl_eq - state.leg_a.dl));
    const double angle_spring_torque_a =
        (env.angle_stiffness * (state.leg_a.theta_eq - state.leg_a.theta)) +
        (env.angle_damping * (state.leg_a.dtheta_eq - state.leg_a.dtheta));
    const double angle_spring_force_a = angle_spring_torque_a / state.leg_a.l;
    const double length_spring_force_b =
        (env.length_stiffness * (state.leg_b.l_eq - state.leg_b.l)) +
        (env.length_damping * (state.leg_b.dl_eq - state.leg_b.dl));
    const double angle_spring_torque_b =
        (env.angle_stiffness * (state.leg_b.theta_eq - state.leg_b.theta)) +
        (env.angle_damping * (state.leg_b.dtheta_eq - state.leg_b.dtheta));
    const double angle_spring_force_b = angle_spring_torque_b / state.leg_b.l;

    // Get basis vectors for internal spring forces
    // Positive when acting on the foot, negate for body
    const double l_x_a = std::sin(state.leg_a.theta + state.phi);
    const double l_y_a = -std::cos(state.leg_a.theta + state.phi);
    const double theta_x_a = -l_y_a;
    const double theta_y_a = l_x_a;
    const double l_x_b = std::sin(state.leg_b.theta + state.phi);
    const double l_y_b = -std::cos(state.leg_b.theta + state.phi);
    const double theta_x_b = -l_y_b;
    const double theta_y_b = l_x_b;

    // Forces on body
    const double force_body_x = ext.body.x -
        (l_x_a * length_spring_force_a) - (theta_x_a * angle_spring_force_a) -
        (l_x_b * length_spring_force_b) - (theta_x_b * angle_spring_force_b);
    const double force_body_y = ext.body.y -
        (l_y_a * length_spring_force_a) - (theta_y_a * angle_spring_force_a) -
        (l_y_b * length_spring_force_b) - (theta_y_b * angle_spring_force_b);
    const double torque_body_phi = ext.body_phi -
        angle_motor_gap_torque_a - angle_motor_gap_torque_b -
        ((1.0 - 1.0 / env.angle_motor_ratio) *
         (angle_spring_torque_a + angle_spring_torque_b));

    // Acceleration of body
    const double ddx = force_body_x / env.body_mass;
    const double ddy = force_body_y / env.body_mass;
    const double ddphi = torque_body_phi / env.body_inertia;

    // Acceleration of leg equilibrium positions
    const double ddtheta_eq_a = (angle_motor_gap_torque_a -
                               angle_spring_torque_a / env.angle_motor_ratio) /
        (env.angle_motor_ratio * env.angle_motor_inertia);
    const double ddl_eq_a = (length_motor_gap_torque_a -
                           length_spring_force_a / env.length_motor_ratio) /
        (env.length_motor_ratio * env.length_motor_inertia);
    const double ddtheta_eq_b = (angle_motor_gap_torque_b -
                               angle_spring_torque_b / env.angle_motor_ratio) /
        (env.angle_motor_ratio * env.angle_motor_inertia);
    const double ddl_eq_b = (length_motor_gap_torque_b -
                           length_spring_force_b / env.length_motor_ratio) /
        (env.length_motor_ratio * env.length_motor_inertia);

    // Convert external forces on foot to relative polar coordinate acceleration
    // Gravity is included in the external forces
    const double accel_offset_foot_x_a = ext.foot_a.x / env.foot_mass - ddx;
    const double accel_offset_foot_y_a = ext.foot_a.y / env.foot_mass - ddy;
    const double accel_foot_l_a = (length_spring_force_a / env.foot_mass) +
        (accel_offset_foot_x_a * l_x_a) + (accel_offset_foot_y_a * l_y_a);
    const double accel_foot_theta_a = (angle_spring_force_a / env.foot_mass) +
        (accel_offset_foot_x_a * theta_x_a) +
        (accel_offset_foot_y_a * theta_y_a);
    const double accel_offset_foot_x_b = ext.foot_b.x / env.foot_mass - ddx;
    const double accel_offset_foot_y_b = ext.foot_b.y / env.foot_mass - ddy;
    const double accel_foot_l_b = (length_spring_force_b / env.foot_mass) +
        (accel_offset_foot_x_b * l_x_b) + (accel_offset_foot_y_b * l_y_b);
    const double accel_foot_theta_b = (angle_spring_force_b / env.foot_mass) +
        (accel_offset_foot_x_b * theta_x_b) +
        (accel_offset_foot_y_b * theta_y_b);

    // Acceleration of actual leg positions
    const double dtheta_abs_a = state.leg_a.dtheta + state.dphi;
    const double ddl_a = accel_foot_l_a +
        (state.leg_a.l * dtheta_abs_a * dtheta_abs_a);
    const double ddtheta_a =
        (accel_foot_theta_a - (2 * state.leg_a.dl * dtheta_abs_a)) /
        state.leg_a.l - ddphi;
    const double dtheta_abs_b = state.leg_b.dtheta + state.dphi;
    const double ddl_b = accel_foot_l_b +
        (state.leg_b.l * dtheta_abs_b * dtheta_abs_b);
    const double ddtheta_b =
        (accel_foot_theta_b - (2 * state.leg_b.dl * dtheta_abs_b)) /
        state.leg_b.l - ddphi;

    // Output state derivative vector
    return {state.dx,
            state.dy,
            state.dphi,
            ddx,
            ddy,
            ddphi,
            {state.leg_a.dl,
             state.leg_a.dl_eq,
             state.leg_a.dtheta,
             state.leg_a.dtheta_eq,
             ddl_a,
             ddl_eq_a,
             ddtheta_a,
             ddtheta_eq_a},
            {state.leg_b.dl,
             state.leg_b.dl_eq,
             state.leg_b.dtheta,
             state.leg_b.dtheta_eq,
             ddl_b,
             ddl_eq_b,
             ddtheta_b,
             ddtheta_eq_b}};
}


inline Force ground_contact_model(PointState point, const Environment& env)
{
    // Find the point on the ground closest to the point to test
    double min_dist2 = std::numeric_limits<double>::max();
    double min_p = 0.0;
    double min_x_line = 0.0;
    double min_y_line = 0.0;
    double min_seg_length2 = 0.0;
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
        const double dot_product = (point.x - xg) * dxg + (point.y - yg) * dyg;
        const double seg_length2 = (dxg * dxg) + (dyg * dyg);
        const double p = clamp(dot_product / seg_length2, 0, 1);

        // Nearest point on the line segment to the test point
        const double x_line = xg + (p * dxg);
        const double y_line = yg + (p * dyg);

        // Squared distance from line point to test point
        const double dist2 = ((point.x - x_line) * (point.x - x_line)) +
            ((point.y - y_line) * (point.y - y_line));

        // If this is a new minimum, save values
        // Ignore segments with zero length
        if (dist2 < min_dist2 && seg_length2 > 0.0)
        {
            min_dist2 = dist2;
            min_p = p;
            min_x_line = x_line;
            min_y_line = y_line;
            min_seg_length2 = seg_length2;
            min_index = i;
        }
    }

    // Check whether point is on the ground side (right hand side) of the line
    // If not, return immediately with zero ground reaction force
    const double dxg = env.ground[min_index + 1].x - env.ground[min_index].x;
    const double dyg = env.ground[min_index + 1].y - env.ground[min_index].y;
    const double dxp = point.x - env.ground[min_index].x;
    const double dyp = point.y - env.ground[min_index].y;
    const double cross_product = (dxg * dyp) - (dyg * dxp);
    if (cross_product > 0.0)
        return {0.0, 0.0};

    // If the point is a vertex, also check the next line
    if (min_p == 1.0 && min_index < env.ground.size() - 2)
    {
        const double dxg = env.ground[min_index + 2].x -
            env.ground[min_index + 1].x;
        const double dyg = env.ground[min_index + 2].y -
            env.ground[min_index + 1].y;
        const double dxp = point.x - env.ground[min_index + 1].x;
        const double dyp = point.y - env.ground[min_index + 1].y;
        const double cross_product = (dxg * dyp) - (dyg * dxp);
        if (cross_product > 0.0)
            return {0.0, 0.0};
    }

    // If execution reaches here, the point is in the ground
    // Note that if the test point is outside the bounds of the
    // polyline, it is handled incorrectly

    // Get normal and tangent basis vectors
    // NOTE: Normal is into ground, tangent is 90 deg CCW from normal
    const double depth = std::sqrt(min_dist2);
    double tangent_x, tangent_y, normal_x, normal_y;
    if (min_p == 0.0 || min_p == 1.0)
    {
        // Special case for corners -- normal is aligned with vector
        // from test point to corner
        normal_x = -(point.x - min_x_line) / depth;
        normal_y = -(point.y - min_y_line) / depth;
        tangent_x = normal_y;
        tangent_y = -normal_x;
    }
    else
    {
        // Typical case -- use segment direction for tangent
        const double seg_length = std::sqrt(min_seg_length2);
        tangent_x = dxg / seg_length;
        tangent_y = dyg / seg_length;
        normal_x = -tangent_y;
        normal_y = tangent_x;
    }

    // Get derivative of depth
    const double ddepth = (-normal_x * point.dx) + (-normal_y * point.dy);

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

    // Damping adjustment factor
    const double damping_factor = depth / (depth + env.ground_damping_depth);

    // Normal force (spring + damper) should only be positive upwards
    const double normal_force =
        std::fmax((depth * ground_stiffness) +
                  (ddepth * damping_factor * ground_damping), 0.0);

    // Tangent force (friction) before finding sign and smoothing discontinuity
    const double friction_max = ground_friction * normal_force;
    const double tangent_velocity = (tangent_x * point.dx) +
        (tangent_y * point.dy);
    const double viscous_friction_factor =
        clamp(tangent_velocity / (friction_max * env.ground_slip_ramp), -1, 1);
    const double tangent_force = -viscous_friction_factor * friction_max;

    return {(normal_x * normal_force) + (tangent_x * tangent_force),
            (normal_y * normal_force) + (tangent_y * tangent_force)};
}


inline LegMotorTorques hardstop_forces(LegState state, const Environment& env)
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


inline PointState foot_state(LegState state, State body_state)
{
    const double l_x = std::sin(state.theta + body_state.phi);
    const double l_y = -std::cos(state.theta + body_state.phi);
    const double theta_x = -l_y;
    const double theta_y = l_x;
    const double dtheta_abs = state.dtheta + body_state.dphi;
    return {
        body_state.x + (l_x * state.l),
        body_state.y + (l_y * state.l),
        body_state.dx + (l_x * state.dl) + (theta_x * state.l * dtheta_abs),
        body_state.dy + (l_y * state.dl) + (theta_y * state.l * dtheta_abs)};
}


inline DState hopper_dynamics(State state,
                              double t,
                              const Environment& env,
                              ControllerTarget target,
                              ControllerParams params,
                              const ControllerState& cstate)
{
    // Get motor torques from low-level controller
    const MotorTorques motors =
        low_level_controller_output(state, t, env, target, params, cstate);

    // Get hardstop forces
    const MotorTorques hardstops =
        {hardstop_forces(state.leg_a, env), hardstop_forces(state.leg_b, env)};

    // Calculate external forces
    const PointState foot_point_a = foot_state(state.leg_a, state);
    const Force foot_ground_force_a = ground_contact_model(foot_point_a, env);
    const PointState foot_point_b = foot_state(state.leg_b, state);
    const Force foot_ground_force_b = ground_contact_model(foot_point_b, env);
    const Force foot_gravity = {0.0, -env.foot_mass * env.gravity};

    const ExternalForces ext = {
        {0.0, -(env.body_mass * env.gravity)},
        0.0,
        foot_ground_force_a + foot_gravity,
        foot_ground_force_b + foot_gravity};

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
    // Update the state of the low-level controller once per major step
    low_level_controller_update(ts.state, ts.time, dt,
                                env, target, params, cstate);

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
                            ControllerParams params,
                            ControllerState& cstate)
{
    // Initialize output vector
    StateSeries output = {{0.0, initial}};

    // Step simulation forward until next timestep will put it over stop time
    while (output.back().time + env.dt < stop_time)
    {
        output.push_back(integration_step(output.back(), env.dt,
                                          env, target, params, cstate));

        // End this step when signalled by the controller
        if (cstate.end_step)
            break;
    }

    return output;
}


} // namespace sim
