classdef autopilot < handle
    properties
        ts_control
        AP
        roll_from_aileron
        course_from_roll
        sideslip_from_rudder
        yaw_damper
        pitch_from_elevator
        altitude_from_pitch
        airspeed_from_throttle
        altitude_zone
        commanded_state
    end

    methods
        %% ---------- Constructor ----------
        function self = autopilot(ts_control)
            global AP
            addpath('../chap6');
            self.ts_control = ts_control;
            self.AP         = AP;

            % ----- Lateral -----
            self.roll_from_aileron = pid_control( ...
                AP.roll_kp, 0, AP.roll_kd, ts_control, deg2rad(45));
            self.course_from_roll  = pid_control( ...
                AP.course_kp, AP.course_ki, 0, ts_control, deg2rad(30));
            self.sideslip_from_rudder = pid_control( ...
                AP.sideslip_kp, AP.sideslip_ki, 0, ts_control, deg2rad(45));
            self.yaw_damper = tf([0.5, 0], 1.0, ts_control);  % not used yet

            % ----- Longitudinal -----
            self.pitch_from_elevator = pid_control( ...
                AP.pitch_kp, 0, AP.pitch_kd, ts_control, deg2rad(45));
            self.altitude_from_pitch = pid_control( ...
                AP.altitude_kp, AP.altitude_ki, 0, ts_control, deg2rad(30));
            self.airspeed_from_throttle = pid_control( ...
                AP.airspeed_throttle_kp, AP.airspeed_throttle_ki, ...
                0, ts_control, 1.0, 0.2, true);

            self.altitude_zone = AP.altitude_zone;

            addpath('../message_types');
            self.commanded_state = msg_state();
        end

        %% ---------- Update ----------
        function [delta, commanded_state] = update(self, cmd, state)
            % ===== Lateral Autopilot =====
            % course -> commanded roll
            phi_c   = self.course_from_roll.update(cmd.course_command, ...
                                                   state.chi, true);
            % roll -> aileron
            delta_a = self.roll_from_aileron.update_with_rate(phi_c, ...
                                                              state.phi, ...
                                                              state.p);
            % sideslip -> rudder
            delta_r = self.sideslip_from_rudder.update(0, state.beta);

            % ===== Longitudinal Autopilot =====
            % altitude -> commanded pitch
            h_c     = self.saturate(cmd.altitude_command, ...
                                    state.h - self.altitude_zone, ...
                                    state.h + self.altitude_zone);
            theta_c = self.altitude_from_pitch.update(h_c, state.h);

            % pitch -> elevator
            delta_e = self.pitch_from_elevator.update_with_rate(theta_c, ...
                                                                state.theta, ...
                                                                state.q);
            % airspeed -> throttle
            delta_t = self.airspeed_from_throttle.update( ...
                            cmd.airspeed_command, state.Va);

            % ===== Pack outputs =====
            delta = [delta_e; delta_t; delta_a; delta_r];

            self.commanded_state.h     = h_c;
            self.commanded_state.Va    = cmd.airspeed_command;
            self.commanded_state.phi   = phi_c;
            self.commanded_state.theta = theta_c;
            self.commanded_state.chi   = cmd.course_command;

            commanded_state = self.commanded_state;
        end

        %% ---------- Saturation helper ----------
        function out = saturate(self, in, low, high)
            out = min(max(in, low), high);
        end
    end
end

