#pragma once

#include <array>
#include <vector>
#include <cstddef>
#include <initializer_list>
#include <cassert>

namespace sim
{


/*******************************************************************************
 * State vector representations
 ******************************************************************************/

struct LegState
{
    // Degrees of freedom
    double l;        // Length of the leg, from body (x, y) to toe
    double l_eq;     // Length of the leg with an uncompressed spring
    double theta;    // Relative angle of the leg, 0 + 0 phi is straight down
    double theta_eq; // Neutral position of the leg angle spring

    // Derivatives
    double dl;
    double dl_eq;
    double dtheta;
    double dtheta_eq;
};


// Representation of state vector as a struct with named members
struct State
{
    // Degrees of freedom
    double x;        // Horizontal position of the body center of mass
    double y;        // Vertical position of the body center of mass
    double phi;      // Absolute angle of the body, CCW positive

    // Derivatives
    double dx;
    double dy;
    double dphi;

    // Legs
    LegState leg_a;
    LegState leg_b;
};


/*******************************************************************************
 * Timeseries of states
 ******************************************************************************/

struct TimeState
{
    double time;
    State state;
};

typedef std::vector<TimeState> StateSeries;


/*******************************************************************************
 * State vector derivative typedefs (for clearer semantics)
 ******************************************************************************/

typedef State    DState;
typedef LegState DLegState;


/*******************************************************************************
 * Vector math shorthands for State/LegState and double
 * Only operations that make sense are implemented
 ******************************************************************************/

// (LegState, LegState)
#define VEC_FUNCTION_LSLS(symbol)                               \
    inline LegState operator symbol (LegState a, LegState b)    \
    {                                                           \
        return {a.l         symbol b.l,                         \
                a.l_eq      symbol b.l_eq,                      \
                a.theta     symbol b.theta,                     \
                a.theta_eq  symbol b.theta_eq,                  \
                a.dl        symbol b.dl,                        \
                a.dl_eq     symbol b.dl_eq,                     \
                a.dtheta    symbol b.dtheta,                    \
                a.dtheta_eq symbol b.dtheta_eq};                \
    }

VEC_FUNCTION_LSLS(+)
VEC_FUNCTION_LSLS(-)


// (LegState, LegState)
#define VEC_FUNCTION_LSD(symbol)                                \
    inline LegState operator symbol (LegState a, double b)      \
    {                                                           \
        return {a.l         symbol b,                           \
                a.l_eq      symbol b,                           \
                a.theta     symbol b,                           \
                a.theta_eq  symbol b,                           \
                a.dl        symbol b,                           \
                a.dl_eq     symbol b,                           \
                a.dtheta    symbol b,                           \
                a.dtheta_eq symbol b};                          \
    }

VEC_FUNCTION_LSD(*)
VEC_FUNCTION_LSD(/)


// (double, LegState)
#define VEC_FUNCTION_DLS(symbol)                                \
    inline LegState operator symbol (double a, LegState b)      \
    {                                                           \
        return {a symbol b.l,                                   \
                a symbol b.l_eq,                                \
                a symbol b.theta,                               \
                a symbol b.theta_eq,                            \
                a symbol b.dl,                                  \
                a symbol b.dl_eq,                               \
                a symbol b.dtheta,                              \
                a symbol b.dtheta_eq};                          \
    }

VEC_FUNCTION_DLS(*)


// (State, State)
#define VEC_FUNCTION_SS(symbol)                                 \
    inline State operator symbol (State a, State b)             \
    {                                                           \
        return {a.x         symbol b.x,                         \
                a.y         symbol b.y,                         \
                a.phi       symbol b.phi,                       \
                a.dx        symbol b.dx,                        \
                a.dy        symbol b.dy,                        \
                a.dphi      symbol b.dphi,                      \
                a.leg_a     symbol b.leg_a,                     \
                a.leg_b     symbol b.leg_b};                    \
    }

VEC_FUNCTION_SS(+)
VEC_FUNCTION_SS(-)


// (State, double)
#define VEC_FUNCTION_SD(symbol)                                 \
    inline State operator symbol (State a, double b)            \
    {                                                           \
        return {a.x         symbol b,                           \
                a.y         symbol b,                           \
                a.phi       symbol b,                           \
                a.dx        symbol b,                           \
                a.dy        symbol b,                           \
                a.dphi      symbol b,                           \
                a.leg_a     symbol b,                           \
                a.leg_b     symbol b};                          \
    }

VEC_FUNCTION_SD(*)
VEC_FUNCTION_SD(/)


// (double, State)
#define VEC_FUNCTION_DS(symbol)                                 \
    inline State operator symbol (double a, State b)            \
    {                                                           \
        return {a symbol b.x,                                   \
                a symbol b.y,                                   \
                a symbol b.phi,                                 \
                a symbol b.dx,                                  \
                a symbol b.dy,                                  \
                a symbol b.dphi,                                \
                a symbol b.leg_a,                               \
                a symbol b.leg_b};                              \
    }

VEC_FUNCTION_DS(*)


} // namespace sim
