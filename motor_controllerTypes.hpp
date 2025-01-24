#ifndef motor_controller_TYPES_HPP
#define motor_controller_TYPES_HPP

#include <base/Float.hpp>
#include <base/JointState.hpp>
#include <motor_controller/PID.hpp>

namespace motor_controller {
    /** Configuration of the controller for a single actuator */
    struct ActuatorSettings
    {
        base::JointState::MODE output_mode = base::JointState::UNSET;
        PIDSettings pid;

        /** Max allowed variation in input command in unit/s */
        float ramp = base::infinity<float>();

        /** How the derivative is computed */
        DerivativeMode derivative_mode = Output;
    };
}

#endif

