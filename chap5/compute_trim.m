function [x_trim, u_trim] = compute_trim(mav, Va, gamma, MAV)
    % Va is the desired airspeed (m/s)
    % gamma is the desired flight path angle (radians)
    % R is the desired radius (m) - use (+) for right handed orbit,
    %                                   (-) for left handed orbit

    % define initial state and input
    addpath('../tools');
    state0 = [0; 0; 0; 25; 0; 0; 1; 0; 0; 0; 0; 0; 0];
    delta0 = [0; 0.5; 0; 0];
    x0 = [state0; delta0];
% Define lower and upper bounds
lb = [-inf*ones(13,1); -1; 0; -1; -1];  % State variables and control inputs
ub = [inf*ones(13,1); 1; 1; 1; 1];      % State variables and control inputs

    xstar = fmincon(@trim_objective, x0, [], [],...
        [], [],lb,ub, @trim_constraints, [],...
        mav, Va, gamma, MAV);

    x_trim = xstar(1:13);
    u_trim = xstar(14:17);
    J = trim_objective(xstar, mav, Va, gamma, MAV);
end

% objective function to be minimized
function J = trim_objective(x, mav, Va, gamma, MAV)

    % Extract state variables
    state = x(1:13);

    % Extract control inputs
    delta = x(14:17);

    % Desired trim state derivatives
    desired_trim_state_dot = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

    % Update MAV state
    mav.state = state;
    mav.update_velocity_data();

    % Compute forces and moments
    forces_moments = mav.forces_moments(delta, MAV);

    % Compute derivatives
    f = mav.derivatives(state, forces_moments, MAV);

    % Compute cost function (minimizing deviation from desired trim state)
    temp = desired_trim_state_dot - f;

    if any(isnan(temp(3:13)))
        error('NaN encountered in temp(3:13), check forces_moments and derivatives calculations.');
    end

    J = norm(temp(3:13))^2;
end

% nonlinear constraints for trim optimization
function [c, ceq] = trim_constraints(x, mav, Va, gamma, MAV)
    % Extract state variables
    u = x(4);
    v = x(5);
    w = x(6);
    e0 = x(7);
    e1 = x(8);
    e2 = x(9);
    e3 = x(10);
    p = x(11);
    q = x(12);
    r = x(13);

    % nonlinear equality constraints (ceq)
    ceq = [];

    % Constraint: Velocity magnitude matches desired airspeed
    ceq(end+1) = u^2 + v^2 + w^2 - Va^2;

    % Constraint: Side velocity (force zero sideslip)
    ceq(end+1) = v;

    % Quaternion normalization constraint (ensuring unit quaternion)
    ceq(end+1) = e0^2 + e1^2 + e2^2 + e3^2 - 1;

    % Quaternion constraints for zero roll and yaw in trim
    ceq(end+1) = e1; % Force e1 to zero for level trim
    ceq(end+1) = e3; % Force e3 to zero for level trim

    % Angular rate constraints (ensuring p = q = r = 0)
    ceq(end+1) = p; % Roll rate
    ceq(end+1) = q; % Pitch rate
    ceq(end+1) = r; % Yaw rate

    % No equality constraints (c)
    c = [];
end

