#include "controller.hpp"

namespace sim
{


MotorTorques low_level_controller(State state,
                                  double t,
                                  ControllerTarget target,
                                  ControllerParams params)
{
    // Silence unused argument warnings
    state = {};
    target = {};
    params = {};
    return {t*0, 0};
}


} // namespace sim
