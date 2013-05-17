#ifndef motor_controller_TYPES_HPP
#define motor_controller_TYPES_HPP

#include <base/actuators/commands.h>
#include <motor_controller/PID.hpp>

namespace motor_controller {

    /** Configuration of the controller for a single actuator */
    struct ActuatorSettings
    {
        base::actuators::DRIVE_MODE output_domain;
        motor_controller::PIDSettings pid;
        bool use_external;
    };
}

#endif

