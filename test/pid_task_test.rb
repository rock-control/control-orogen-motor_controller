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

    describe 'input validations' do
        before do
            settings = Types.motor_controller.ActuatorSettings.new
            settings.zero!
            settings.output_mode = :EFFORT
            @task.properties.settings = [settings]
            syskit_configure_and_start(@task)
        end

        it 'emits invalid_input_command if the command has no fields set' do
            command = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Effort(Float::NAN)]
            )
            status = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Position(0)]
            )

            expect_execution do
                syskit_write task.in_command_port, command
                syskit_write task.status_samples_port, status
            end.to do
                emit task.invalid_input_command_event
            end
        end

        it 'emits invalid_input_command if the command has more than one field set' do
            state = Types.base.JointState.Effort(0)
            state.position = 0
            command = Types.base.samples.Joints.new(
                elements: [state]
            )
            status = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Position(0)]
            )

            expect_execution do
                syskit_write task.in_command_port, command
                syskit_write task.status_samples_port, status
            end.to do
                emit task.invalid_input_command_event
            end
        end

        it 'validates that the expected input field is set in the status sample' do
            command = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Effort(0)]
            )
            status = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Position(0)]
            )

            expect_execution do
                syskit_write task.in_command_port, command
                syskit_write task.status_samples_port, status
            end.to do
                emit task.invalid_status_sample_event
            end
        end
    end

    it 'does not carry old values if the output mode changes' do
        settings = Types.motor_controller.ActuatorSettings.new
        settings.zero!
        settings.output_mode = :RAW
        @task.properties.settings = [settings]
        syskit_configure_and_start(@task)

        command = Types.base.samples.Joints.new(
            elements: [Types.base.JointState.Raw(0.1)]
        )
        status = Types.base.samples.Joints.new(
            elements: [Types.base.JointState.Raw(0.2)]
        )

        expect_execution do
            syskit_write task.in_command_port, command
            syskit_write task.status_samples_port, status
        end.to do
            have_one_new_sample task.out_command_port
        end

        task.needs_reconfiguration!
        @task = syskit_deploy(
            OroGen.motor_controller.PIDTask
                  .deployed_as('pid_task_test')
        )

        settings.output_mode = :EFFORT
        # Garbage collect the old task to be able to configure and start the
        # new one
        expect_execution { @task.properties.settings = [settings.dup] }
            .garbage_collect(true)
            .to_run
        syskit_configure_and_start(task)

        cmd = expect_execution do
            syskit_write task.in_command_port, command
            syskit_write task.status_samples_port, status
        end.to do
            have_one_new_sample task.out_command_port
        end

        refute cmd.elements[0].effort.nan?
        assert cmd.elements[0].raw.nan?
    end

    describe 'dynamic property changes' do
        attr_reader :settings

        before do
            @settings = Types.motor_controller.ActuatorSettings.new
            settings.zero!
            settings.output_mode = :RAW
            settings.pid.K = 1
            settings.pid.B = 1
            @task.properties.settings = [settings]
            syskit_configure_and_start(@task)
        end

        it 'accepts dynamic changes to the PID parameters' do
            settings.pid.K = 0.1
            execute { @task.properties.settings = [settings.dup] }

            command = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Raw(0.1)]
            )
            status = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Raw(0.2)]
            )

            cmd = expect_execution do
                syskit_write task.in_command_port, command
                syskit_write task.status_samples_port, status
            end.to do
                have_one_new_sample task.out_command_port
            end

            assert_in_delta(-0.01, cmd.elements[0].raw, 1e-3)
        end

        it 'rejects changes that also change the count of channels' do
            settings = Types.motor_controller.ActuatorSettings.new
            settings.zero!
            expect_execution { @task.properties.settings = [settings, settings] }
                .to do
                    have_error_matching(
                        Syskit::PropertyUpdateError.match.with_origin(task)
                    )
                end
        end

        it 'does not carry old values if the output mode changes' do
            command = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Raw(0.1)]
            )
            status = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Raw(0.2)]
            )

            cmd = expect_execution do
                syskit_write task.in_command_port, command
                syskit_write task.status_samples_port, status
            end.to do
                have_one_new_sample task.out_command_port
            end

            settings.output_mode = :EFFORT
            execute { @task.properties.settings = [settings.dup] }
            cmd = expect_execution do
                syskit_write task.status_samples_port, status
            end.to do
                have_one_new_sample task.out_command_port
            end

            refute cmd.elements[0].effort.nan?
            assert cmd.elements[0].raw.nan?
        end
    end
end
