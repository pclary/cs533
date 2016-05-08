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

// Representation of state vector as an array
typedef std::array<double, 14> StateVec;


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


    // These constructors are usually provided by default, but adding
    // the conversion from StateVec constructor removes the implicit
    // constructors
    State() = default;
    State(std::initializer_list<double> il)
    {
        assert(il.size() <= sizeof (State));
        size_t i = 0;
        for (double d : il)
            (*reinterpret_cast<StateVec*>(this))[i++] = d;
    }


    // Methods for converting to and from array form
    State(const StateVec& sv)
    {
        *reinterpret_cast<StateVec*>(this) = sv;
    }

    operator StateVec() const
    {
        return *reinterpret_cast<const StateVec*>(this);
    }
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
typedef StateVec DStateVec;


/*******************************************************************************
 * Vector math shorthands for (StateVec, StateVec)
 ******************************************************************************/

inline StateVec operator+(StateVec a, StateVec b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a[i] + b[i];
    return out;
}

inline StateVec operator-(StateVec a, StateVec b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a[i] - b[i];
    return out;
}

inline StateVec operator*(StateVec a, StateVec b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a[i] * b[i];
    return out;
}

inline StateVec operator/(StateVec a, StateVec b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a[i] / b[i];
    return out;
}


/*******************************************************************************
 * Vector math shorthands for (StateVec, double)
 ******************************************************************************/

inline StateVec operator+(StateVec a, double b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a[i] + b;
    return out;
}

inline StateVec operator-(StateVec a, double b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a[i] - b;
    return out;
}

inline StateVec operator*(StateVec a, double b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a[i] * b;
    return out;
}

inline StateVec operator/(StateVec a, double b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a[i] / b;
    return out;
}


/*******************************************************************************
 * Vector math shorthands for (double, StateVec)
 ******************************************************************************/

inline StateVec operator+(double a, StateVec b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a + b[i];
    return out;
}

inline StateVec operator-(double a, StateVec b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a - b[i];
    return out;
}

inline StateVec operator*(double a, StateVec b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a * b[i];
    return out;
}

inline StateVec operator/(double a, StateVec b)
{
    StateVec out;
    for (size_t i = 0; i < out.size(); ++i)
        out[i] = a / b[i];
    return out;
}


} // namespace sim
