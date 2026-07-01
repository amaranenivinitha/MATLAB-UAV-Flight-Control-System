function control_parameters()
% Sets autopilot gains in global struct AP using TF coefficients

global AP MAV Va ...
       a_phi1 a_phi2 a_theta_1 a_theta_2 a_theta_3 ...
       a_V_1 a_V_2 a_beta_1 a_beta_2

AP.gravity = MAV.gravity;
AP.sigma   = 0.05;
AP.Va0     = Va;

%% ========= ROLL LOOP (HEAVILY DAMPED) =========
wn_phi   = 3.0;         % natural freq [rad/s]
zeta_phi = 1.8;         % high damping

AP.roll_kp =  wn_phi^2 / a_phi2;
AP.roll_kd = (2*zeta_phi*wn_phi - a_phi1) / a_phi2;

%% ========= COURSE LOOP (SLOWER) =========
W_chi  = 20;              % separation (>5)
wn_chi = wn_phi / W_chi;

AP.course_kp = 2*zeta_phi*wn_chi*Va / AP.gravity;
AP.course_ki =    wn_chi^2 * Va     / AP.gravity;

%% ========= PITCH LOOP =========
wn_theta   = 7.0;         % rad/s
zeta_theta = 0.9;

AP.pitch_kp = (wn_theta^2 - a_theta_2) / a_theta_3;
AP.pitch_kd = (2*zeta_theta*wn_theta - a_theta_1) / a_theta_3;

K_theta_DC = AP.pitch_kp * a_theta_3 / wn_theta^2;

%% ========= ALTITUDE LOOP =========
W_h   = 6;
wn_h  = wn_theta / W_h;

AP.altitude_kp   = 2*zeta_theta*wn_h / (K_theta_DC * Va);
AP.altitude_ki   =    wn_h^2         / (K_theta_DC * Va);
AP.altitude_zone = 10;   % [m] deadband around commanded altitude

%% ========= AIRSPEED HOLD (THROTTLE) =========
wn_v   = 1.2;
zeta_v = 0.8;

AP.airspeed_throttle_kp = (2*zeta_v*wn_v - a_V_1) / a_V_2;
AP.airspeed_throttle_ki =  wn_v^2                 / a_V_2;

%% ========= SIDESLIP / RUDDER =========
AP.sideslip_kp = -4 * a_beta_2 / a_beta_1;  % strong damping
AP.sideslip_ki = 0.05;                      % small integral

end






