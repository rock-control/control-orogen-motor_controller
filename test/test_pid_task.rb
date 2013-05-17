require 'orocos'
require 'minitest/spec'
require 'orocos/test'

describe 'the PID task' do
    include Orocos::Spec

    attr_reader :task, :status_w, :command_w

    before do
        Orocos.load_typekit 'motor_controller'
        start 'motor_controller::PIDTask' => 'task'
        @task = name_service.get 'task'
        @status_w  = task.status_samples.writer
        @command_w = task.in_command.writer
    end

    it "should fail if the expected set of actuators changed after the configure hook" do
        settings = Types::MotorController::ActuatorSettings.new
        settings.zero!
        task.settings = [settings, settings]
        task.configure
        task.settings = [settings]
        assert_raises(Orocos::StateTransitionFailed) { task.start }
    end

    it "should fail if the expected set of actuators do not match the number of entries in the status" do
        settings = Types::MotorController::ActuatorSettings.new
        settings.zero!
        settings.output_domain = :DM_PWM
        task.settings = [settings]
        task.configure
        task.start

        status  = status_w.new_sample
        status.zero!
        command = command_w.new_sample
        command.zero!
        command.mode << :DM_PWM
        command.target << 0
        command_w.write(command)
        status_w.write(status)
        sleep 0.1
        assert_state_equals :WRONG_INPUT_SIZE, task
    end

    it "should fail if the expected set of actuators do not match the number of entries in the command" do
        settings = Types::MotorController::ActuatorSettings.new
        settings.zero!
        settings.output_domain = :DM_PWM
        task.settings = [settings]
        task.configure
        task.start

        status  = status_w.new_sample
        status.zero!
        status.states << Types::Base::Actuators::MotorState.new
        command = command_w.new_sample
        command.zero!
        command_w.write(command)
        status_w.write(status)
        sleep 0.1
        assert_state_equals :WRONG_INPUT_SIZE, task
    end
end

