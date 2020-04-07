function M = compute_sym_m(dh_table, sigma)
%COMPUTE_SYM_M Computes Inertia Matrix M through Lagrange-Euler method:
%  dh_table : [nx4] Denavit Hartenberg table of the arm
%  sigma : [nx1] prismatic flag vector
%  Returns:
%  M : [nxn] Inertia matrix


%  The algorithm recursively computes the links kinetic energies from 
%  which it interpolates the inertial matrix.
%  The function is based on slides:
%  http://www.diag.uniroma1.it/deluca/rob2_en/04_LagrangianDynamics_2.pdf
%  page: 3

% Most of the functions are based on Robotics2Tools provided by MoSamArafat
% https://github.com/MoSamArafat/Robotics2Tools

% Made by Emanuele Giacomini (2020)

[n, ~] = size(dh_table);
end