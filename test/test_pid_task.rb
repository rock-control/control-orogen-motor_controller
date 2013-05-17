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

    describe "application of the PID controller" do
        attr_reader :status, :settings, :command, :command_r
        before do
            @settings = Types::MotorController::ActuatorSettings.new
            settings.zero!
            settings.pid.K = 1
            @status  = status_w.new_sample
            status.zero!
            status.states << Types::Base::Actuators::MotorState.new
            status.states[0].zero!
            @command = command_w.new_sample
            command.mode << :DM_UNINITIALIZED
            command.target << 0
            @command_r = task.out_command.reader
        end

        def do_test_input_field_selection(mode, position, positionExtern, pwm)
            task.settings = [settings]
            task.configure
            task.start
            command.mode[0] = mode
            command_w.write command
            status.states[0].position = position
            status.states[0].positionExtern = positionExtern
            status.states[0].pwm = pwm
            status_w.write status
            output = read_one_sample(command_r)
            assert_equal(0, output.target[0])
        end

        it "should use the PWM field when in PWM mode" do
            do_test_input_field_selection(:DM_PWM, 1, 1, 0)
        end
        it "should use the position field when in internal position mode" do
            settings.use_external = false
            do_test_input_field_selection(:DM_POSITION, 0, 1, 1)
        end
        it "should use the position field when in external position mode" do
            settings.use_external = true
            do_test_input_field_selection(:DM_POSITION, 1, 0, 1)
        end
    end
end

