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

    describe 'the application of the PID controller' do
        def do_test_input_field_selection(
            mode, position, speed, effort, raw, acceleration
        )
            settings = Types.motor_controller.ActuatorSettings.new
            settings.zero!
            settings.output_mode = mode
            settings.pid.B = 1
            settings.pid.K = 0.01
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

            assert_in_delta -0.1, cmd.elements[0][mode], 1e-3
        end

        it 'uses the position field when in position mode' do
            do_test_input_field_selection(:POSITION, 10, 1, 1, 1, 1)
        end
        it 'uses the speed field when in speed mode' do
            do_test_input_field_selection(:SPEED, 1, 10, 1, 1, 1)
        end
        it 'uses the effort field when in effort' do
            do_test_input_field_selection(:EFFORT, 1, 1, 10, 1, 1)
        end
        it 'uses the raw field when in rawmode' do
            do_test_input_field_selection(:RAW, 1, 1, 1, 10, 1)
        end
        it 'uses the acceleration field when in acceleration mode' do
            do_test_input_field_selection(:ACCELERATION, 1, 1, 1, 1, 10)
        end

        it 'propagates the names from the input command' do
            settings = Types.motor_controller.ActuatorSettings.new
            settings.zero!
            settings.output_mode = :EFFORT
            @task.properties.settings = [settings]
            syskit_configure_and_start(@task)

            command = Types.base.samples.Joints.new(
                names: ['somethingsomething'],
                elements: [Types.base.JointState.Effort(0)]
            )
            status = Types.base.samples.Joints.new(
                elements: [Types.base.JointState.Effort(0)]
            )

            cmd = expect_execution do
                syskit_write task.in_command_port, command
                syskit_write task.status_samples_port, status
            end.to do
                have_one_new_sample task.out_command_port
            end

            assert_equal ['somethingsomething'], cmd.names
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
                        Syskit::PropertyUpdatesError.match.with_origin(task)
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

    describe "ramp" do
        before do
            @settings = Types.motor_controller.ActuatorSettings.new
            @settings.zero!
            @settings.output_mode = :EFFORT
            @settings.pid.K = 1
            @settings.pid.Ts = 1
            @settings.pid.B = 1

            @command = Types.base.samples.Joints.new(
                time: Time.now,
                elements: [Types.base.JointState.Effort(0)]
            )
            @status = Types.base.samples.Joints.new(
                time: Time.now,
                elements: [Types.base.JointState.Effort(0)]
            )
        end

        it "does not do any positive limiting if the ramp is infinite" do
            @settings.ramp = Float::INFINITY
            _, cmd_writer = configure_and_start(@task)

            write_cmd_effort cmd_writer, 1_000_000

            cmd = expect_execution do
                syskit_write @task.status_samples_port, @status
            end.to { have_one_new_sample task.out_command_port }
            assert_in_delta 1_000_000, cmd.elements[0].effort, 1e-3
        end

        it "does not do any negative limiting if the ramp is infinite" do
            @settings.ramp = Float::INFINITY
            _, cmd_writer = configure_and_start(@task)

            write_cmd_effort cmd_writer, -1_000_000

            cmd = expect_execution do
                syskit_write @task.status_samples_port, @status
            end.to { have_one_new_sample task.out_command_port }
            assert_in_delta -1_000_000, cmd.elements[0].effort, 1e-3
        end

        it "ramps up the control input according to the rate set in the settings" do
            @settings.ramp = 10
            _, cmd_writer = configure_and_start(@task)
            start_time = Time.now

            write_cmd_effort cmd_writer, 100

            10.times do
                before = Time.now
                cmd = expect_execution do
                    syskit_write @task.status_samples_port, @status
                end.to { have_one_new_sample task.out_command_port }
                after = Time.now

                assert_in_interval(
                    cmd.elements[0].effort,
                    (before - start_time) * 10 * 0.9,
                    (after - start_time) * 10 * 1.1
                )
                sleep 0.2
            end
        end

        it "ramps down the control input according to the rate set in the settings" do
            @settings.ramp = 10
            _, cmd_writer = configure_and_start(@task)
            start_time = Time.now

            write_cmd_effort cmd_writer, -100

            10.times do
                before = Time.now
                cmd = expect_execution do
                    syskit_write @task.status_samples_port, @status
                end.to { have_one_new_sample task.out_command_port }
                after = Time.now

                assert_in_interval(
                    cmd.elements[0].effort,
                    (after - start_time) * -10 * 1.1,
                    (before - start_time) * -10 * 0.9
                )
                sleep 0.2
            end
        end

        it "ramps from the last generated command on command update" do
            @settings.ramp = 10
            _, cmd_writer = configure_and_start(@task)
            start_time = Time.now

            write_cmd_effort cmd_writer, 100
            sleep 0.1

            before = Time.now
            cmd = expect_execution do
                syskit_write @task.status_samples_port, @status
            end.to { have_one_new_sample task.out_command_port }
            after = Time.now

            assert_in_interval(
                cmd.elements[0].effort,
                (before - start_time) * 10 * 0.9,
                (after - start_time) * 10 * 1.1
            )

            start_cmd = cmd.elements[0].effort
            write_cmd_effort cmd_writer, (start_cmd + 0.05)
            sleep 0.1

            cmd = expect_execution do
                syskit_write @task.status_samples_port, @status
            end.to { have_one_new_sample task.out_command_port }
            assert_in_delta start_cmd + 0.05, cmd.elements[0].effort, 1e-4
        end

        it "sticks to the target once reached (positive)" do
            @settings.ramp = 10
            _, cmd_writer = configure_and_start(@task)

            write_cmd_effort cmd_writer, 10
            sleep 1.1

            cmd = expect_execution do
                syskit_write @task.status_samples_port, @status
            end.to { have_one_new_sample task.out_command_port }
            assert_equal 10, cmd.elements[0].effort
        end

        it "sticks to the target once reached (negative)" do
            @settings.ramp = 10
            _, cmd_writer = configure_and_start(@task)

            write_cmd_effort cmd_writer, -10
            sleep 1.1

            cmd = expect_execution do
                syskit_write @task.status_samples_port, @status
            end.to { have_one_new_sample task.out_command_port }
            assert_equal -10, cmd.elements[0].effort
        end

        def configure_and_start(task)
            task.properties.settings = [@settings]
            syskit_configure_and_start(task)

            cmd_writer = @task.in_command_port.writer
            expect_execution.to { achieve { cmd_writer.ready? } }
            cmd_writer.write(@command)
            sleep 0.1

            sample = expect_execution do
                syskit_write task.status_samples_port, @status
            end.to { have_one_new_sample task.out_command_port }

            [sample, cmd_writer]
        end

        def write_cmd_effort(writer, effort)
            @command.elements[0].effort = effort
            writer.write @command
            sleep 0.1
        end

        def assert_in_interval(value, min, max)
            flunk("#{value} to be in [#{min}, #{max}]") if value <= min
            flunk("#{value} to be in [#{min}, #{max}]") if value >= max
        end
    end
end
