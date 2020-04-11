function [c, s] = compute_christoffel(M, q, dq)
%COMPUTE_CHRISTOFFEL Compute the Christoffel symbols and the skew matrix factorization.
%  M  : [nxn] Inertia matrix
%  q  : [nx1] column vector of joints coordinates
%  dq : [nx1] column vector of joints velocities   
%  Returns:
%  c : [nx1] centrifugal/coriolis term
%  s : [nxn] skew symmetric factorization for c
%
%  q = [q1;q2;...];
%  dq = [dq1;dq2;...];
%  [c, S] = compute_christoffel(M, q, dq)

% Most of the functions are based on Robotics2Tools provided by MoSamArafat
% https://github.com/MoSamArafat/Robotics2Tools

% Made by Emanuele Giacomini (2020)

% no. of joints
[n, ~]= size(M);

% precompute the sine/cosine 's for the n joints
% used to collect the factors on final results
sin_mat = sym(zeros(1, n));
cos_mat = sym(zeros(1, n));
for i=1:n
    sin_mat(i) = sin(q(i));
    cos_mat(i) = cos(q(i));
end
vars = [sin_mat, cos_mat];

% pre-allocate the output matrices
% c: christoffel terms
% s: skew-symmetric matrix
c = sym(zeros(n, 1));
s = sym(zeros(n, n));

% compute the christoffel symbols based on slides:
% 03_LagrangianDytnamics_1.pdf page: 22
for i=1:n
    % column i of M
    mi = M(:, i);
    % derivative of mi w.r.t. q
    dmi_dq = collect(simplify(jacobian(mi, q)), vars);
    % compute the C_k(q) term (k-th component of vector c)
    fprintf('[Christoffel] C_%i\n', i)
    c_i = collect(simplify(0.5 * (dmi_dq + dmi_dq' - diff(M, q(i)))), vars);
    disp(c_i);
    c(i, 1) = collect(simplify(dq' * c_i * dq), vars);
    % compute the skew factorization matrix based on slides:
    % 03_LagrangianDytnamics_1.pdf page: 28
    s(i, :) = collect(simplify(c_i * dq) , vars);
end
end