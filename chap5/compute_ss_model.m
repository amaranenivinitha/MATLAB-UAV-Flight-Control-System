function [A_lon,B_lon,A_lat,B_lat] = compute_ss_model(mav, x_trim, u_trim, MAV)

  if length(x_trim) == 13
      x_euler = euler_state(x_trim);
  end

  E1 = [0 0 0 1 0 0 0 0 0 0 0 0;...
        0 0 0 0 0 1 0 0 0 0 0 0;...
        0 0 0 0 0 0 0 0 0 0 1 0;...
        0 0 0 0 0 0 0 1 0 0 0 0;...
        0 0 -1 0 0 0 0 0 0 0 0 0];

  E2 = [1 0 0 0; 0 1 0 0];

  A = df_dx(mav, x_euler, u_trim, MAV);
  B = df_du(mav, x_euler, u_trim, MAV);
  A_lon = E1*A*E1';
  B_lon = E1*B*E2';

  E3 = [0 0 0 0 1 0 0 0 0 0 0 0;...
        0 0 0 0 0 0 0 0 0 1 0 0;...
        0 0 0 0 0 0 0 0 0 0 0 1;...
        0 0 0 0 0 0 0 0 1 0 0 0;...
        0 0 0 0 0 0 1 0 0 0 0 0];

  E4 = [0 0 1 0; 0 0 0 1];

  A_lat = E3*A*E3';
  B_lat = E3*B*E4';
end

% convert state x with attitude represented by quaternion
% to x_euler with attitude represented by Euler angles
function x_euler = euler_state(x_quat)
  pn = x_quat(1);
  pe = x_quat(2);
  pd = x_quat(3);
  u = x_quat(4);
  v = x_quat(5);
  w = x_quat(6);
  e0 = x_quat(7);
  e1 = x_quat(8);
  e2 = x_quat(9);
  e3 = x_quat(10);
  p = x_quat(11);
  q = x_quat(12);
  r = x_quat(13);

  % Convert quaternion to Euler angles
  phi = atan2(2*(e0*e1 + e2*e3), 1 - 2*(e1^2 + e2^2));
  theta = asin(2*(e0*e2 - e3*e1));
  psi = atan2(2*(e0*e3 + e1*e2), 1 - 2*(e2^2 + e3^2));

  x_euler = [pn; pe; pd; u; v; w; psi; theta; phi; p; q; r];
end

% convert state x_euler with attitude represented by Euler
% angles to x_quat with attitude represented by quaternions
function x_quat = quaternion_state(x_trim)
  pn = x_euler(1);
  pe = x_euler(2);
  pd = x_euler(3);
  u = x_euler(4);
  v = x_euler(5);
  w = x_euler(6);
  psi = x_euler(7);
  theta = x_euler(8);
  phi = x_euler(9);
  p = x_euler(10);
  q = x_euler(11);
  r = x_euler(12);

% Convert Euler angles to quaternion
e0 = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
e1 = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
e2 = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
e3 = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2);

x_quat = [pn; pe; pd; u; v; w; e0; e1; e2; e3; p; q; r];

end

% return 12x1 dynamics (as if state were Euler state)
% compute f at euler_state
function xdot = f_euler(mav, x_euler, u_trim, MAV)
    % extract states from x_euler
    pn = x_euler(1);
    pe = x_euler(2);
    pd = x_euler(3);
    u = x_euler(4);
    v = x_euler(5);
    w = x_euler(6);
    psi = x_euler(7);
    theta = x_euler(8);
    phi = x_euler(9);
    p = x_euler(10);
    q = x_euler(11);
    r = x_euler(12);
   
force = mav.forces_moments(u_trim, MAV);
fx = force(1);
fy = force(2);
fz = force(3);
l = force(4);
m = force(5);
n = force(6);

% Compute xdot
pn_dot = u * cos(theta) * cos(psi) + v * (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) + w * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
pe_dot = u * cos(theta) * sin(psi) + v * (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) + w * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
pd_dot = -u * sin(theta) + v * sin(phi) * cos(theta) + w * cos(phi) * cos(theta);

Rb2v = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);...
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)*cos(psi);...
        -sin(theta),        sin(phi)*cos(theta),                          cos(phi)*cos(theta)];

pos_dot = Rb2v * [u; v; w];
pn_dot = pos_dot(1);
pe_dot = pos_dot(2);
pd_dot = pos_dot(3);
u_dot = (r*v - q*w) + fx / MAV.mass;
v_dot = (p*w - r*u) + fy / MAV.mass;
w_dot = (q*u - p*v) + fz / MAV.mass;
e0_dot = 0; 
e1_dot = 0; 
e2_dot = 0; 
e3_dot = 0;

Rdotb2v = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
                          0, cos(phi), -sin(phi);...
                          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
eul_dot = Rdotb2v * [p; q; r];
    psi_dot = eul_dot(1);
    theta_dot = eul_dot(2);
    phi_dot = eul_dot(3);

    p_dot = (MAV.Gamma1*p*q - MAV.Gamma2*q*r) + (MAV.Gamma3*l + MAV.Gamma4*n);
    q_dot = (MAV.Gamma5*p*r) - MAV.Gamma6*(p^2 - r^2) + m / MAV.Jy;
    r_dot = (MAV.Gamma7*p*q - MAV.Gamma1*q*r) + (MAV.Gamma4*l + MAV.Gamma8*n);

    % Collect derivatives into xdot
    xdot = [pn_dot; pe_dot; pd_dot; u_dot; v_dot; w_dot;
            psi_dot; theta_dot; phi_dot; p_dot; q_dot; r_dot];
end

% Compute the state-space model for the MAV system
% x_trim - trim state
% u_trim - trim input
% MAV - structure containing MAV properties

% take partial of f_euler with respect to x_euler
function A = df_dx(mav, x_euler, u_trim, MAV)
    epsilon = 1e-5; % small perturbation for numerical derivatives
    num_states = length(x_euler);
    A = zeros(num_states); % Initialize A matrix

    % compute A matrix
    for i = 1:num_states
        x_perturb = x_euler;
        x_perturb(i) = x_perturb(i) + epsilon;

        f_plus = f_euler(mav, x_perturb, u_trim, MAV);
        f_minus = f_euler(mav, x_euler, u_trim, MAV);

        A(:, i) = (f_plus - f_minus) / epsilon;
    end
end

% take partial of f_euler with respect to delta
function B = df_du(mav, x_euler, u_trim, MAV)
    epsilon = 1e-5; % small perturbation for numerical derivatives
    num_states = length(x_euler);
    num_inputs = length(u_trim);
    B = zeros(num_states, num_inputs); % Initialize B matrix

    % compute B matrix
    for j = 1:num_inputs
        u_perturb = u_trim;
        u_perturb(j) = u_perturb(j) + epsilon;

        f_plus = f_euler(mav, x_euler, u_perturb, MAV);
        f_minus = f_euler(mav, x_euler, u_trim, MAV);

        B(:, j) = (f_plus - f_minus) / epsilon;
    end
end