function [T_phi_delta_a, ...
          T_chi_phi, ...
          T_theta_delta_e, ...
          T_h_theta, ...
          T_h_Va, ...
          T_Va_delta_t, ...
          T_Va_theta, ...
          T_v_delta_r]... %T_beta_delta_r 
          = compute_tf_model(x_trim, u_trim, P)

s = tf('s'); % Define Laplace variable
Q_inf = MAV.rho * Va^2 * MAV.S_wing / 2; % Dynamic pressure at the wing

% phi_delta_a
C_p_p = MAV.Gamma3 * MAV.C_ell_p + MAV.Gamma4 * MAV.C_n_p;
C_p_delta_a = MAV.Gamma3 * MAV.C_ell_delta_a + MAV.Gamma4 * MAV.C_n_delta_a;
a_phi1 = -Q_inf * MAV.b * C_p_p * (MAV.b / (2 * Va));
a_phi2 = Q_inf * MAV.b * C_p_delta_a;
T_phi_delta_a = (a_phi1 / (s * (s + a_phi1)));

% chi_phi
T_chi_phi = (MAV.gravity / Va) / s;

% theta_delta_e
a_theta1 = (-Q_inf * MAV.c / MAV.Jy) * MAV.C_m_q * (MAV.c / (2 * Va));
a_theta2 = (-Q_inf * MAV.c / MAV.Jy) * MAV.C_m_alpha;
a_theta3 = (Q_inf * MAV.c / MAV.Jy) * MAV.C_m_delta_e;
T_theta_delta_e = a_theta3 / (s^2 + a_theta1 * s + a_theta2);

% h_Va
T_h_Va = Va / s;

% h_theta
T_h_theta = theta / s; % Need to check

% Va_delta_t & Va_theta
u_star = x_star(4);
v_star = x_star(5);
w_star = x_star(6);
Va_star = sqrt(u_star^2 + v_star^2 + w_star^2);
alpha_star = atan2(u_star, w_star);
delta_e_star = u_trim(1);
delta_t_star = u_trim(2);
theta_star = alpha_star + gamma_star;

a_V1 = (MAV.rho * Va_star^2 * MAV.S_wing / MAV.mass) * (MAV.C_D_0 + MAV.C_D_alpha * alpha_star + MAV.C_D_delta_e * delta_e_star) + (MAV.rho * MAV.S_prop * MAV.C_prop * Va_star / MAV.mass);
a_V2 = (MAV.rho * MAV.S_prop * MAV.C_prop * MAV.k_motor^2 * delta_t_star) / MAV.mass;
a_V3 = MAV.gravity * cos(theta_star - alpha_star);

T_Va_delta_t = a_V2 / (s + a_V1);
T_Va_theta = -a_v3 / (s + a_v1);

% beta_delta_r
a_beta1 = (-MAV.rho * Va * MAV.S_wing / (2 * MAV.mass)) * MAV.C_Y_beta;
a_beta2 = (MAV.rho * Va * MAV.S_wing / (2 * MAV.mass)) * MAV.C_Y_delta_r;
T_beta_delta_r = a_beta2 / (s + a_beta1);

end

% returns the derivative of motor thrust with respect to Va
function dThrust = dT_dVa(mav, Va, delta_t)
dThrust = (-MAV.rho * MAV.S_prop * MAV.C_prop * Va);
end

% returns the derivative of motor thrust with respect to delta_t
function dThrust = dT_ddelta_t(mav, Va, delta_t)
dThrust = (MAV.rho * MAV.S_prop * MAV.C_prop * MAV.k_motor^2*delta_t);
end