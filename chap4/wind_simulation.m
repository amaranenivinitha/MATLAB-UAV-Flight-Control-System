classdef wind_simulation < handle
    %====================================
    properties
        steady_state % Steady-state wind vector [wn; we; wd]
        A % State-space matrix A for gust model
        B % State-space matrix B for gust model
        C % State-space matrix C for gust model
        gust_state % Gust model state vector
        gust_ % Current gust vector
        Ts % Sample time
    end
    %====================================
    methods
        %-----constructor-----
        function self = wind_simulation(Ts)
            % Define parameters
            Va = 20; % Airspeed (m/s)
            Lu = 200; Lv = 200; Lw = 50; % Turbulence scale lengths (m)
            sigma_u = 1.06; sigma_v = 2.12; sigma_w = 1.4; % Turbulence intensities (m/s)
            
            % Transfer function coefficients for Dryden gust model
            Hu = sigma_u * sqrt(2*Va / (pi*Lu)) * tf([1, 2*Va/Lu], ...
                                                      [1, 2*Va/Lu, (Va/Lu)^2]);
            Hv = sigma_v * sqrt(3*Va / (pi*Lv)) * tf([1, Va/(sqrt(3)*Lv)], ...
                                                      [1, 2*Va/Lv, (Va/Lv)^2]);
            Hw = sigma_w * sqrt(3*Va / (pi*Lw)) * tf([1, Va/(sqrt(3)*Lw)], ...
                                                      [1, 2*Va/Lw, (Va/Lw)^2]);

            % Convert transfer functions to state-space
            [A_u, B_u, C_u, ~] = tf2ss(Hu.Numerator{1}, Hu.Denominator{1});
            [A_v, B_v, C_v, ~] = tf2ss(Hv.Numerator{1}, Hv.Denominator{1});
            [A_w, B_w, C_w, ~] = tf2ss(Hw.Numerator{1}, Hw.Denominator{1});
            
            % Combine state-space matrices
            self.A = blkdiag(A_u, A_v, A_w);
            self.B = blkdiag(B_u, B_v, B_w);
            self.C = blkdiag(C_u, C_v, C_w);
            
            % Initialize properties
            self.steady_state = [0; 0; 0]; % Example steady-state wind [wn; we; wd]
            self.gust_state = zeros(size(self.A, 1), 1);
            self.gust_ = [0; 0; 0];
            self.Ts = Ts;
        end
        
        %----------------------------------
        function wind = update(self)
            self.gust();
            % Returns the total wind vector (steady-state + gust)
            wind = [self.steady_state; self.gust_];
        end
        
        %----------------------------------
        function self = gust(self)
            % Update gust using state-space equations
            w = randn(3, 1); % White noise inputs for u, v, w
            self.gust_state = (eye(size(self.A)) + self.Ts * self.A) * self.gust_state + ...
                              self.Ts * self.B * w;
            self.gust_ = self.C * self.gust_state;
        end
    end
end