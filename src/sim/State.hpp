#pragma once

namespace sim
{


struct State
{
    double x;
    double y;
    double theta;
    double theta_eq;
    double l;
    double l_eq;

    double dx;
    double dy;
    double dtheta;
    double dtheta_eq;
    double dl;
    double dl_eq;
};


struct DState
{
    double dx;
    double dy;
    double dtheta;
    double dtheta_eq;
    double dl;
    double dl_eq;

    double ddx;
    double ddy;
    double ddtheta;
    double ddtheta_eq;
    double ddl;
    double ddl_eq;
};


} // namespace sim
