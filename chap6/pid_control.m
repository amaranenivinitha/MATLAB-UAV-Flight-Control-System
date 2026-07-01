classdef pid_control < handle
    properties
        kp
        ki
        kd
        Ts
        limit
        throttle_flag
        tau
        integrator
        y_dot
        y_dl
        error_dot
        error_dl
        a1
        a2
    end

    methods
        % --------- Constructor ---------
        function self = pid_control(kp, ki, kd, Ts, limit, sigma, throttle_flag)
            if nargin < 7, throttle_flag = false; end
            if nargin < 6, sigma         = 0.2;   end
            if nargin < 5, limit         = 1.0;   end

            self.kp = kp;
            self.ki = ki;
            self.kd = kd;
            self.Ts = Ts;
            self.limit = limit;
            self.throttle_flag = throttle_flag;
            self.tau = sigma;

            self.integrator = 0.0;
            self.y_dot      = 0.0;
            self.y_dl       = 0.0;
            self.error_dot  = 0.0;
            self.error_dl   = 0.0;

            self.a1 = (2*self.tau - self.Ts) / (2*self.tau + self.Ts);
            self.a2 = 2 / (2*self.tau + self.Ts);
        end

        % --------- Update (with optional wrap for angles) ---------
        function u_sat = update(self, y_ref, y, wrap_angle)
            if nargin < 4, wrap_angle = false; end

            error = y_ref - y;

            if wrap_angle
                while error > pi
                    error = error - 2*pi;
                end
                while error < -pi
                    error = error + 2*pi;
                end
            end

            self.integrateError(error);
            self.differentiateY(y);

            u_unsat = self.kp*error + self.ki*self.integrator - self.kd*self.y_dot;
            u_sat   = self.saturate(u_unsat);
            self.antiWindup(u_sat, u_unsat);
        end

        % --------- Update with rate feedback ---------
        function u_sat = update_with_rate(self, y_ref, y, ydot)
            error = y_ref - y;
            self.integrateError(error);

            u_unsat = self.kp*error + self.ki*self.integrator - self.kd*ydot;
            u_sat   = self.saturate(u_unsat);
            self.antiWindup(u_sat, u_unsat);
        end

        % --------- Helpers ---------
        function integrateError(self, error)
            self.integrator = self.integrator + self.Ts/2 * (error + self.error_dl);
            self.error_dl   = error;
        end

        function differentiateY(self, y)
            self.y_dot = self.a1*self.y_dot + self.a2*(y - self.y_dl);
            self.y_dl  = y;
        end

        function u_sat = saturate(self, u)
            if u >= self.limit
                u_sat = self.limit;
            elseif self.throttle_flag && (u <= 0.0)
                u_sat = 0.0;
            elseif u <= -self.limit
                u_sat = -self.limit;
            else
                u_sat = u;
            end
        end

        function antiWindup(self, u_sat, u_unsat)
            if self.ki ~= 0
                self.integrator = self.integrator + self.Ts/self.ki * (u_sat - u_unsat);
            end
        end
    end
end
