#include "controller.hpp"

namespace sim
{


MotorTorques low_level_controller(State state,
                                  double t,
                                  ControllerTarget target,
                                  ControllerParams params)
{
    return {0, 0};
}


} // namespace sim
