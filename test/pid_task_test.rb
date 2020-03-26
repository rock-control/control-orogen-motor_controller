# frozen_string_literal: true

using_task_library 'motor_controller'

describe OroGen.motor_controller.PIDTask do
    run_live

    attr_reader :task
    before do
        @task = syskit_deploy(
            OroGen.motor_controller.PIDTask
                  .deployed_as('pid_task_test')
        )
    end

    describe 'input validation' do
        before do
            settings = Types.motor_controller.ActuatorSettings.new
            settings.zero!
            settings.output_mode = :RAW
            @task.properties.settings = [settings]
            syskit_configure_and_start(@task)
        end

        it 'fails if the expected set of actuators do not match '\
        'the number of entries in the status' do
            command = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.new]
            )
            status = Types.base.samples.Joints.new

            expect_execution do
                syskit_write task.in_command_port, command
                syskit_write task.status_samples_port, status
            end.to do
                emit task.wrong_status_size_event
            end
        end

        it 'fails if the expected set of actuators do not match '\
        'the number of entries in the command' do
            command = Types.base.samples.Joints.new
            status = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.new]
            )

            expect_execution do
                syskit_write task.in_command_port, command
                syskit_write task.status_samples_port, status
            end.to do
                emit task.wrong_input_command_size_event
            end
        end
    end

    describe 'the application of the PID controller' do
        def do_test_input_field_selection(
            mode, position, speed, effort, raw, acceleration
        )
            settings = Types.motor_controller.ActuatorSettings.new
            settings.zero!
            settings.output_mode = mode
            @task.properties.settings = [settings]
            syskit_configure_and_start(@task)

            command = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.new]
            )
            command.elements[0][mode] = 0
            status = Types.base.samples.Joints.new(
                elements: [
                    Types.base.JointState.new(
                        position: position,
                        speed: speed,
                        effort: effort,
                        raw: raw,
                        acceleration: acceleration
                    )
                ]
            )

            cmd = expect_execution do
                syskit_write task.in_command_port, command
                syskit_write task.status_samples_port, status
            end.to do
                have_one_new_sample task.out_command_port
            end

            assert_equal 0, cmd.elements[0][mode]
        end

        it 'uses the position field when in position mode' do
            do_test_input_field_selection(:POSITION, 0, 1, 1, 1, 1)
        end
        it 'uses the speed field when in speed mode' do
            do_test_input_field_selection(:SPEED, 1, 0, 1, 1, 1)
        end
        it 'uses the effort field when in effort' do
            do_test_input_field_selection(:EFFORT, 1, 1, 0, 1, 1)
        end
        it 'uses the raw field when in rawmode' do
            do_test_input_field_selection(:RAW, 1, 1, 1, 0, 1)
        end
        it 'uses the acceleration field when in acceleration mode' do
            do_test_input_field_selection(:ACCELERATION, 1, 1, 1, 1, 0)
        end
    end
end
