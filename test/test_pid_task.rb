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
        command.states << Types::Base::JointState.new
        command_w.write(command)
        status_w.write(status)
        sleep 0.1
        assert_state_equals :WRONG_STATUS_SIZE, task
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
        status.states << Types::Base::JointState.new
        command = command_w.new_sample
        command.zero!
        command_w.write(command)
        status_w.write(status)
        sleep 0.1
        assert_state_equals :WRONG_INPUT_COMMAND_SIZE, task
    end

    describe "application of the PID controller" do
        attr_reader :status, :settings, :command, :command_r
        before do
            @settings = Types::MotorController::ActuatorSettings.new
            settings.zero!
            settings.pid.K = 1
            @status  = status_w.new_sample
            status.zero!
            status.states << Types::Base::JointState.new
            @command = command_w.new_sample
            command.states << Types::Base::JointState.new
            @command_r = task.out_command.reader
        end

        def do_test_input_field_selection(mode, position, speed, effort, raw)
            task.settings = [settings]
            task.configure
            task.start
            command.states[0][mode] = 0
            command_w.write command
            status.states[0] = Types::Base::JointState.new(
                :position => position,
                :speed => speed,
                :effort => effort,
                :raw => raw)
            status_w.write status
            output = read_one_sample(command_r)
            assert_equal(0, output.target[0])
        end

        it "should use the position field when in position mode" do
            do_test_input_field_selection(:POSITION, 0, 1, 1, 1)
        end
        it "should use the speed field when in speed mode" do
            do_test_input_field_selection(:SPEED, 1, 0, 1, 1)
        end
        it "should use the effort field when in effort" do
            do_test_input_field_selection(:EFFORT, 1, 1, 0, 1)
        end
        it "should use the raw field when in rawmode" do
            do_test_input_field_selection(:RAW, 1, 1, 1, 0)
        end
    end
end

