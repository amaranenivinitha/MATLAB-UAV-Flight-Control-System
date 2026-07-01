function q = Euler2Quaternion(phi, theta, psi)
    % Converts Euler angles (phi, theta, psi) to a quaternion [e0, e1, e2, e3]
    q0 = cos(phi/2) * cos(theta/2) * cos(psi/2) + sin(phi/2) * sin(theta/2) * sin(psi/2);
    q1 = sin(phi/2) * cos(theta/2) * cos(psi/2) - cos(phi/2) * sin(theta/2) * sin(psi/2);
    q2 = cos(phi/2) * sin(theta/2) * cos(psi/2) + sin(phi/2) * cos(theta/2) * sin(psi/2);
    q3 = cos(phi/2) * cos(theta/2) * sin(psi/2) - sin(phi/2) * sin(theta/2) * cos(psi/2);
    q = [q0; q1; q2; q3];
end
