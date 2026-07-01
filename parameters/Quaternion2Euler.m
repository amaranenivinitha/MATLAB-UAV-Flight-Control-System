function euler = Quaternion2Euler(q)

e0 = q(1);
e1 = q(2);
e2 = q(3);
e3 = q(4);

phi = atan2(2*(e0*e1 + e2*e3), ...
    1 - 2*(e1^2 + e2^2));

theta = asin(2*(e0*e2 - e3*e1));

psi = atan2(2*(e0*e3 + e1*e2), ...
    1 - 2*(e2^2 + e3^2));

euler = [phi; theta; psi];

end