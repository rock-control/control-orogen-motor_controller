/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef MOTOR_CONTROLLER_PIDTASK_TASK_HPP
#define MOTOR_CONTROLLER_PIDTASK_TASK_HPP

#include "motor_controller/PIDTaskBase.hpp"
#include <base/commands/Joints.hpp>

namespace motor_controller {
    /*! \class PIDTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','motor_controller::PIDTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class PIDTask : public PIDTaskBase
    {
    protected:
        bool setSettings(std::vector<motor_controller::ActuatorSettings> const & value);

        std::vector<ActuatorSettings> mSettings;
        std::vector<motor_controller::PID> mPIDs;
        base::samples::Joints mStatus;
        base::commands::Joints mInputCommand;
        base::commands::Joints mOutputCommand;
        std::vector<motor_controller::PIDState> mPIDState;

        typedef std::pair<base::Time, float> RampState;
        std::vector<RampState> mRampState;
        float applyRamp(int idx, base::Time time, float input);

        /** Computes an output command
         *
         * The default implementation simply calls the PID controller for the
         * requested actuator
         *
         * @param idx the index of the actuator we want to compute the command of
         * @param output_domain the requested output domain
         * @param state the current state of the actuator
         * @param target the requested target for this actuator (output_domain
         *   describes how this value should be interpreted)
         * @param now the current time
         */
        virtual float computePIDOutput(int idx,
                base::JointState::MODE output_domain,
                float state, float target, base::Time now);

    public:
        /** TaskContext constructor for PIDTask
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        PIDTask(std::string const& name = "motor_controller::PIDTask");

        /** TaskContext constructor for PIDTask
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        PIDTask(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of PIDTask
         */
	~PIDTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

