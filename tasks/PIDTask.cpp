/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PIDTask.hpp"

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
    if (! PIDTaskBase::configureHook())
        return false;

    size_t size =_settings.get().size();
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
    if (! PIDTaskBase::startHook())
        return false;

    if (_settings.get().size() != mPIDs.size())
    {
        RTT::log(RTT::Error) << "changed the size of the settings property without re-running configure()" << RTT::endlog();
        return false;
    }

    std::vector<motor_controller::ActuatorSettings> const& settings = _settings.get();
    for (size_t i = 0; i < settings.size(); ++i)
    {
        mPIDs[i].reset();
        mPIDs[i].setPIDSettings(settings[i].pid);
    }
    return true;
}
void PIDTask::updateHook()
{
    PIDTaskBase::updateHook();

    if (_in_command.read(mInputCommand) == RTT::NoData)
        return;
    if (mInputCommand.size() != mPIDs.size())
        return exception(WRONG_INPUT_COMMAND_SIZE);

    // Do something only when we have a new status sample
    if (_status_samples.read(mStatus) != RTT::NewData)
        return;
    if (mStatus.size() != mPIDs.size())
        return exception(WRONG_STATUS_SIZE);

    for (size_t i = 0; i < mStatus.size(); ++i)
    {
        JointState::MODE input_domain = mInputCommand[i].getMode();
        float input_target = mInputCommand[i].getField(input_domain);
        float input_state  = mStatus[i].getField(input_domain);

        ActuatorSettings const& settings(_settings.get()[i]);
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
