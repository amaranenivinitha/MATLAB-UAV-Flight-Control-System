function transfer_function_coef()
% Computes linear TF coefficients around trimmed flight condition

global MAV Va x_trim u_trim ...
       a_phi1 a_phi2 a_theta_1 a_theta_2 a_theta_3 ...
       a_V_1 a_V_2 a_beta_1 a_beta_2

% --- extract trim body velocities ---
u = x_trim(4);
v = x_trim(5);
w = x_trim(6);

Va = sqrt(u^2 + v^2 + w^2);                    % airspeed
Q  = 0.5 * MAV.rho * Va^2 * MAV.S_wing;        % dynamic pressure

%% ===== Roll dynamics (phi from delta_a) =====
C_p_p        = MAV.Gamma3 * MAV.C_ell_p       + MAV.Gamma4 * MAV.C_n_p;
C_p_delta_a  = MAV.Gamma3 * MAV.C_ell_delta_a + MAV.Gamma4 * MAV.C_n_delta_a;

a_phi1 = -Q * MAV.b^2 * C_p_p       / (2 * Va);
a_phi2 =  Q * MAV.b    * C_p_delta_a;

%% ===== Pitch dynamics (theta from delta_e) =====
a_theta_1 = (-Q * MAV.c / MAV.Jy) * MAV.C_m_q      * MAV.c / (2*Va);
a_theta_2 = (-Q * MAV.c / MAV.Jy) * MAV.C_m_alpha;
a_theta_3 = ( Q * MAV.c / MAV.Jy) * MAV.C_m_delta_e;

%% ===== Airspeed dynamics from throttle =====
delta_t_trim = u_trim(2);

% Simple linearization: dVa/dt ≈ a_V_1 * Va + a_V_2 * delta_t
a_V_1 = -Q * MAV.C_D_0 / MAV.mass;   % approximate drag slope
a_V_2 = (MAV.rho * MAV.S_prop * MAV.C_prop * MAV.k_motor^2 * delta_t_trim) ...
        / MAV.mass;

%% ===== Sideslip / yaw dynamics =====
a_beta_1 = -Q * MAV.C_Y_beta   / (2 * MAV.mass);
a_beta_2 =  Q * MAV.C_Y_delta_r/ (2 * MAV.mass);

end




