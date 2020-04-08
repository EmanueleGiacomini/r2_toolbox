function G = compute_gravity(q, mass, roc, g0)
%COMPUTE_GRAVITY Compute the robot's gravity term.
%  q    : [nx1] column vector of joints coordinates
%  mass : [nx1] Link mass vector
%  roc  : [nx3] CoM offset matrix wrt RF0
%  g0   : [3x1] Gravity vector wrt RF0
%  Returns:
%  G : [nx1] gravity vector 

[n, ~] = size(mass);

U = 0;
for i=1:n
    ui = -mass(i) * g0' * roc(:, i);
    U = U + ui;
end

G = jacobian(U, q)';
end

