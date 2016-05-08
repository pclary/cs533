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

// Representation of state vector as a struct with named members
struct State
{
    // Degrees of freedom
    double x;        // Horizontal position of the body center of mass
    double y;        // Vertical position of the body center of mass
    double phi;      // Absolute angle of the body, CCW positive
    double l;        // Length of the leg, from body (x, y) to toe
    double l_eq;     // Length of the leg with an uncompressed spring
    double theta;    // Relative angle of the leg, 0 + 0 phi is straight down
    double theta_eq; // Neutral position of the leg angle spring

    // Derivatives
    double dx;
    double dy;
    double dphi;
    double dl;
    double dl_eq;
    double dtheta;
    double dtheta_eq;
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

typedef State DState;


/*******************************************************************************
 * Vector math shorthands State and double
 * Only operations that make sense are implemented
 ******************************************************************************/

// (State, State)
#define VEC_FUNCTION_SS(symbol)                                 \
    inline State operator symbol (State a, State b)             \
    {                                                           \
        return {a.x         symbol b.x,                         \
                a.y         symbol b.y,                         \
                a.phi       symbol b.phi,                       \
                a.l         symbol b.l,                         \
                a.l_eq      symbol b.l_eq,                      \
                a.theta     symbol b.theta,                     \
                a.theta_eq  symbol b.theta_eq,                  \
                a.dx        symbol b.dx,                        \
                a.dy        symbol b.dy,                        \
                a.dphi      symbol b.dphi,                      \
                a.dl        symbol b.dl,                        \
                a.dl_eq     symbol b.dl_eq,                     \
                a.dtheta    symbol b.dtheta,                    \
                a.dtheta_eq symbol b.dtheta_eq};                \
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
                a.l         symbol b,                           \
                a.l_eq      symbol b,                           \
                a.theta     symbol b,                           \
                a.theta_eq  symbol b,                           \
                a.dx        symbol b,                           \
                a.dy        symbol b,                           \
                a.dphi      symbol b,                           \
                a.dl        symbol b,                           \
                a.dl_eq     symbol b,                           \
                a.dtheta    symbol b,                           \
                a.dtheta_eq symbol b};                          \
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
                a symbol b.l,                                   \
                a symbol b.l_eq,                                \
                a symbol b.theta,                               \
                a symbol b.theta_eq,                            \
                a symbol b.dx,                                  \
                a symbol b.dy,                                  \
                a symbol b.dphi,                                \
                a symbol b.dl,                                  \
                a symbol b.dl_eq,                               \
                a symbol b.dtheta,                              \
                a symbol b.dtheta_eq};                          \
    }

VEC_FUNCTION_DS(*)


} // namespace sim
