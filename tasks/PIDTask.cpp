/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PIDTask.hpp"
#include <base-logging/Logging.hpp>

using namespace motor_controller;
using base::JointState;

PIDTask::PIDTask(std::string const& name)
    : PIDTaskBase(name)
{
}

PIDTask::PIDTask(std::string const& name, RTT::ExecutionEngine* engine)
    : PIDTaskBase(name, engine)
{
}

PIDTask::~PIDTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PIDTask.hpp for more detailed
// documentation about them.

bool PIDTask::configureHook()
{
    if (! PIDTaskBase::configureHook()) {
        return false;
    }

    mSettings = _settings.get();
    size_t size = mSettings.size();
    mPIDs.resize(size);
    mStatus.resize(size);
    mInputCommand.resize(size);
    mOutputCommand.resize(size);
    _out_command.setDataSample(mOutputCommand);
    mPIDState.resize(size);
    _pid_states.setDataSample(mPIDState);
    return true;
}

bool PIDTask::startHook()
{
    if (! PIDTaskBase::startHook()) {
        return false;
    }

    for (size_t i = 0; i < mSettings.size(); ++i) {
        mPIDs[i].reset();
    }
    return true;
}

bool PIDTask::setSettings(std::vector< ::motor_controller::ActuatorSettings > const & value)
{
    if (value.size() != mSettings.size()) {
        LOG_ERROR_S << "cannot change the settings size dynamically" << std::endl;
        return false;
    }

    mSettings = value;
    for (size_t i = 0; i < mSettings.size(); ++i) {
        mPIDs[i].setPIDSettings(mSettings[i].pid);
    }
    return motor_controller::PIDTaskBase::setSettings(value);
}

void PIDTask::updateHook()
{
    PIDTaskBase::updateHook();

    if (_in_command.read(mInputCommand) == RTT::NoData) {
        return;
    }
    if (mInputCommand.size() != mPIDs.size()) {
        return exception(WRONG_INPUT_COMMAND_SIZE);
    }

    // Do something only when we have a new status sample
    if (_status_samples.read(mStatus) != RTT::NewData) {
        return;
    }
    if (mStatus.size() != mPIDs.size()) {
        return exception(WRONG_STATUS_SIZE);
    }

    for (size_t i = 0; i < mStatus.size(); ++i)
    {
        JointState::MODE input_domain = mInputCommand[i].getMode();
        float input_target = mInputCommand[i].getField(input_domain);
        float input_state  = mStatus[i].getField(input_domain);

        if (base::isUnknown(input_state)) {
            LOG_ERROR_S
                << "Status sample does not contain the necessary information"
                << std::endl;
            return exception(INVALID_STATUS_SAMPLE);
        }

        ActuatorSettings const& settings(mSettings[i]);
        JointState::MODE output_domain = settings.output_mode;

        float pid_output = computePIDOutput(i,
                output_domain,
                input_state,
                input_target,
                mStatus.time);

        mOutputCommand[i].setField(output_domain, pid_output);
        mPIDState[i] = mPIDs[i].getState();
    }
    _out_command.write(mOutputCommand);
    _pid_states.write(mPIDState);
}

float PIDTask::computePIDOutput(int idx,
        JointState::MODE output_domain,
        float state,
        float target,
        base::Time now)
{
    return mPIDs[idx].update(state, target, now.toSeconds());
}

void PIDTask::errorHook()
{
    PIDTaskBase::errorHook();
}
void PIDTask::stopHook()
{
    PIDTaskBase::stopHook();
}
void PIDTask::cleanupHook()
{
    PIDTaskBase::cleanupHook();
}
