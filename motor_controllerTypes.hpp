#ifndef motor_controller_TYPES_HPP
#define motor_controller_TYPES_HPP

#include <base/JointState.hpp>
#include <motor_controller/PID.hpp>

namespace motor_controller {
    /** Configuration of the controller for a single actuator */
    struct ActuatorSettings
    {
        base::JointState::MODE output_mode;
        PIDSettings pid;

        ActuatorSettings()
            : output_mode(base::JointState::UNSET) {}
    };
}

#endif

