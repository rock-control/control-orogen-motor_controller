/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PIDTask.hpp"

using namespace motor_controller;
using namespace base::actuators;

void DumbVelocityFilter::reset()
{
    mLastValue = 0;
    mLastTime = base::Time();
}

double DumbVelocityFilter::update(base::Time now, double v)
{
    double result = base::unknown<double>();
    if (!mLastTime.isNull())
        result = (v - mLastValue) / (now - mLastTime).toSeconds();

    mLastValue = v;
    mLastTime = now;
    return result;
}

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
    mStatus.states.resize(size);
    mInputCommand.resize(size);
    mOutputCommand.resize(size);
    _out_command.setDataSample(mOutputCommand);
    mPIDState.resize(size);
    _pid_states.setDataSample(mPIDState);
    mVelocityFilters.resize(size);
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
        mVelocityFilters[i].reset();
        mPIDs[i].setPIDSettings(settings[i].pid);
    }
    return true;
}
void PIDTask::updateHook()
{
    PIDTaskBase::updateHook();

    if (_in_command.read(mInputCommand) == RTT::NoData)
        return;
    if (mInputCommand.target.size() != mPIDs.size() || mInputCommand.mode.size() != mPIDs.size())
        return exception(WRONG_INPUT_SIZE);

    // Do something only when we have a new status sample
    if (_status_samples.read(mStatus) != RTT::NewData)
        return;
    if (mStatus.states.size() != mPIDs.size())
        return exception(WRONG_INPUT_SIZE);

    for (size_t i = 0; i < mStatus.states.size(); ++i)
    {
        DRIVE_MODE input_domain = mInputCommand.mode[i];
        double     input_target = mInputCommand.target[i];
        MotorState const& state(mStatus.states[i]);
        ActuatorSettings const& settings(_settings.get()[i]);
        DRIVE_MODE output_domain = settings.output_domain;

        double pid_input;
        if (DM_UNINITIALIZED == input_domain)
            return exception(UNINITIALIZED_INPUT_MODE);
        else if (DM_PWM == input_domain) // Treat this as RAW
            pid_input = state.pwm;
        else
        {
            if (settings.use_external)
                pid_input = mStatus.states[i].positionExtern;
            else
                pid_input = mStatus.states[i].position;

            if (input_domain == DM_SPEED)
                pid_input = computeSpeedCommand(i, mStatus.time, pid_input);
        }

        // We use unset as a way to tell that we can't send commands yet
        if (base::isUnknown<double>(pid_input))
            return;

        mOutputCommand.mode[i] = output_domain;
        mOutputCommand.target[i] = computePIDOutput(i, output_domain, pid_input, input_target, mStatus.time);
        mPIDState[i] = mPIDs[i].getState();
    }
    _out_command.write(mOutputCommand);
    _pid_states.write(mPIDState);
}

double PIDTask::computeSpeedCommand(int idx, base::Time time, double position)
{
    return mVelocityFilters[idx].update(time, position);
}

double PIDTask::computePIDOutput(int idx, DRIVE_MODE output_domain, double state, double target, base::Time now)
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
