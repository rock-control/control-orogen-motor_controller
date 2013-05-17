/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PIDTask.hpp"

using namespace motor_controller;

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
    return true;
}
bool PIDTask::startHook()
{
    if (! PIDTaskBase::startHook())
        return false;
    return true;
}
void PIDTask::updateHook()
{
    PIDTaskBase::updateHook();
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
