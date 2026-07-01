classdef mav_dynamics < handle
    %--------------------------------------
    properties
        ts_simulation
        state
        Va
        alpha
        beta
        wind
        true_state
    end
    
    methods
        % ------- Constructor --------
        function self = mav_dynamics(Ts, MAV)
            self.ts_simulation = Ts; % time step between function calls
            self.state = [MAV.pn0; MAV.pe0; MAV.pd0; MAV.u0; MAV.v0; MAV.w0; ...
                          MAV.e0; MAV.e1; MAV.e2; MAV.e3; MAV.p0; MAV.q0; MAV.r0];

            self.Va = 25;
            self.alpha = 0;
            self.beta = 0;
            self.wind = [0; 0; 0; 0; 0; 0];
            addpath('../message_types'); 
            self.true_state = msg_state();
        end

        function self = update_state(self, delta, wind, MAV)
            % Integrate the differential equations defining dynamics
            % forces_moments are the forces and moments on the MAV
            
            % Get forces and moments acting on rigid body
            forces_moments = self.forces_moments(delta, MAV);
            
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, forces_moments, MAV);
            k2 = self.derivatives(self.state + self.ts_simulation/2 * k1, forces_moments, MAV);
            k3 = self.derivatives(self.state + self.ts_simulation/2 * k2, forces_moments, MAV);
            k4 = self.derivatives(self.state + self.ts_simulation * k3, forces_moments, MAV);
            
            self.state = self.state + self.ts_simulation/6 * (k1 + 2*k2 + 2*k3 + k4);

            % Normalize the quaternion
            self.state(7:10) = self.state(7:10) / norm(self.state(7:10));

% Update the airspeed, angle of attack, and side slip angles
self.update_velocity_data();

% Update the message class for the true state
self.update_true_state();
end
%--------------------------------------

function xdot = derivatives(self, state, forces_moments, MAV)
    % Placeholder for derivative computation based on state and forces/moments
    % Should calculate xdot based on MAV dynamics equations

    % Extract states for readability
    pn = state(1);
    pe = state(2);
    pd = state(3);
    u = state(4);
    v = state(5);
    w = state(6);
    e0 = state(7);
    e1 = state(8);
    e2 = state(9);
    e3 = state(10);
    p = state(11);
    q = state(12);
    r = state(13);

    % Forces and moments
    fx = forces_moments(1);
    fy = forces_moments(2);
    fz = forces_moments(3);
    l = forces_moments(4);
    m = forces_moments(5);
    n = forces_moments(6);

    % Compute derivatives
    pn_dot = u * (e1^2 + e0^2 - e2^2 - e3^2) + 2 * v * (e1 * e2 - e3 * e0) + 2 * w * (e1 * e3 + e2 * e0);
    pe_dot = 2 * u * (e1 * e2 + e3 * e0) + v * (e2^2 + e0^2 - e1^2 - e3^2) + 2 * w * (e2 * e3 - e1 * e0);
    pd_dot = 2 * u * (e1 * e3 - e2 * e0) + 2 * v * (e2 * e3 + e1 * e0) + w * (e3^2 + e0^2 - e1^2 - e2^2);

    u_dot = (r * v - q * w) + fx / MAV.mass;
    v_dot = (p * w - r * u) + fy / MAV.mass;
    w_dot = (q * u - p * v) + fz / MAV.mass;

    e0_dot = -0.5 * (e1 * p - e2 * q - e3 * r);
    e1_dot = 0.5 * (e0 * p + e2 * r - e3 * q);
    e2_dot = 0.5 * (e0 * q - e1 * r + e3 * p);
    e3_dot = 0.5 * (e0 * r + e1 * q - e2 * p);

    p_dot = (MAV.Gamma1 * p * q - MAV.Gamma2 * q * r) + (MAV.Gamma3 * l + MAV.Gamma4 * n);
    q_dot = (MAV.Gamma5 * p * r) - MAV.Gamma6 * (p^2 - r^2) + m / MAV.Jy;
r_dot = (MAV.Gamma7 * p * q - MAV.Gamma1 * q * r) + (MAV.Gamma4 * l + MAV.Gamma8 * n);

% Collect derivatives into xdot
xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;...
        e0_dot; e1_dot; e2_dot; e3_dot; p_dot; q_dot; r_dot];
end

%--------------------------------------
function self = update_velocity_data(self)
    % Update airspeed, angle of attack, and sideslip based on current state

    % Extract rotation matrix from quaternion
    euler = Quaternion2Euler(self.state(7:10));

    phi = euler(1);
    theta = euler(2);
    psi = euler(3);
    Rb2v = [cos(theta)*cos(psi),  sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi),  cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
            cos(theta)*sin(psi),  sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi),  cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
            -sin(theta),          sin(phi)*cos(theta),                               cos(phi)*cos(theta)];

    % Transform steady wind components to body frame
    steady_wind_body = Rb2v' * self.wind(1:3);

    % Add gust components in body frame
    total_wind_body = steady_wind_body + self.wind(4:6);

    % Compute air-relative velocity
    uvwr = self.state(4:6) - total_wind_body; % V_a_b = V_g_b - V_w_b
    ur = uvwr(1);
    vr = uvwr(2);
    wr = uvwr(3);

    % Compute airspeed, angle of attack, and sideslip
    self.Va = sqrt(ur^2 + vr^2 + wr^2);
    self.alpha = atan2(wr, ur);
    self.beta = asin(vr / self.Va);
end

%--------------------------------------
function out = forces_moments(self, delta, MAV)
    % Compute forces and moments based on control inputs and MAV parameters

    delta_e = delta(1);
    delta_t = delta(2);
    delta_a = delta(3);
    delta_r = delta(4);

    % Calculate coefficients
    CL_alpha = MAV.C_L_0 + MAV.C_L_alpha * self.alpha;
    CD_alpha = MAV.C_D_p + CL_alpha^2 / (pi * MAV.AR);
CX_alpha = -CD_alpha * cos(self.alpha) + CL_alpha * sin(self.alpha);
CXq_alpha = -MAV.C_D_q * cos(self.alpha) + MAV.C_L_q * sin(self.alpha);
CXdeltae_alpha = -MAV.C_D_delta_e * cos(self.alpha) + MAV.C_L_delta_e * sin(self.alpha);

CZ_alpha = -CD_alpha * sin(self.alpha) - CL_alpha * cos(self.alpha);
CZq_alpha = -MAV.C_D_q * sin(self.alpha) - MAV.C_L_q * cos(self.alpha);
CZdeltae_alpha = -MAV.C_D_delta_e * sin(self.alpha) + MAV.C_L_delta_e * cos(self.alpha);

Q = 0.5 * MAV.rho * (self.Va)^2 * MAV.S_wing; % Dynamic pressure

fx_aero = Q * (CX_alpha + CXq_alpha * (MAV.c * self.state(12)/(2*self.Va)) + CXdeltae_alpha * delta_e);
fy_aero = Q * (MAV.C_Y_0 + MAV.C_Y_beta * self.beta + MAV.C_Y_p * (MAV.b * self.state(11)/(2*self.Va)) + MAV.C_Y_r * (MAV.b * self.state(13)/(2*self.Va)) + MAV.C_Y_delta_a * delta_a + MAV.C_Y_delta_r * delta_r);
fz_aero = Q * (CZ_alpha + CZq_alpha * (MAV.c * self.state(12)/(2*self.Va)) + CZdeltae_alpha * delta_e);

fx_prop = 0.5 * MAV.rho * MAV.S_prop * MAV.C_prop * ((MAV.k_motor * delta_t)^2 - (self.Va)^2);
fy_prop = 0;
fz_prop = 0;

l_aero = MAV.C_ell_0 + MAV.C_ell_beta * self.beta + MAV.C_ell_p * (MAV.b * self.state(11)/(2*self.Va)) + MAV.C_ell_r * (MAV.b * self.state(13)/(2*self.Va)) + MAV.C_ell_delta_a * delta_a + MAV.C_ell_delta_r * delta_r;
m_aero = MAV.C_m_0 + MAV.C_m_alpha * self.alpha + MAV.C_m_q * (MAV.c * self.state(12)/(2*self.Va)) + MAV.C_m_delta_e * delta_e;
n_aero = MAV.C_n_0 + MAV.C_n_beta * self.beta + MAV.C_n_p * (MAV.b * self.state(11)/(2*self.Va)) + MAV.C_n_r * (MAV.b * self.state(13)/(2*self.Va)) + MAV.C_n_delta_a * delta_a + MAV.C_n_delta_r * delta_r;

l_prop = -MAV.k_T_P * (MAV.k_Omega * delta_t)^2;
m_prop = 0;
n_prop = 0;

% Calculate forces
fx = -MAV.mass * MAV.gravity * sin(self.true_state.theta) + fx_aero + fx_prop;
fy = MAV.mass * MAV.gravity * cos(self.true_state.theta) * sin(self.true_state.phi) + fy_aero + fy_prop;
fz = MAV.mass * MAV.gravity * cos(self.true_state.theta) * cos(self.true_state.phi) + fz_aero + fz_prop;

% Calculate moments
l = Q * MAV.b * l_aero + l_prop;
m = Q * MAV.c * m_aero + m_prop;
n = Q * MAV.b * n_aero + n_prop;

% Assemble output
Force = [fx; fy; fz];
Torque = [l; m; n];
out = [Force; Torque];

end

% =================================
function self = update_true_state(self)
    euler = Quaternion2Euler(self.state(7:10));
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);

    Rb2v = [cos(theta) * cos(psi), sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi), cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
            cos(theta) * sin(psi), sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi), cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi);
            -sin(theta), sin(phi) * cos(theta), cos(phi) * cos(theta)];

    V_i = Rb2v * self.state(4:6);
    ui = V_i(1);
    vi = V_i(2);
    wi = V_i(3);

    self.true_state.pn = self.state(1); % pn
    self.true_state.pe = self.state(2); % pd
    self.true_state.h = -self.state(3); % h
    self.true_state.phi = phi; % phi
    self.true_state.theta = theta; % theta
    self.true_state.psi = psi; % psi
    self.true_state.p = self.state(11); % p
    self.true_state.q = self.state(12); % q
    self.true_state.r = self.state(13); % r
    self.true_state.Va = self.Va;
    self.true_state.alpha = self.alpha;
    self.true_state.beta = self.beta;
    self.true_state.Vg = norm(self.state(4:6)); % Groundspeed
    self.true_state.chi = atan2(vi, ui); % Course angle
    self.true_state.gamma = asin(-wi / norm([ui; vi; wi])); % Flight path angle
    self.true_state.wn = self.wind(1);
    self.true_state.we = self.wind(2);

end
    end
end


