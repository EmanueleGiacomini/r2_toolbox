function M = compute_num_m(dh_table, sigma)
%COMPUTE_NUM_M Computes Inertia Matrix M through Newton-Euler method.
%  dh_table : [nx4] Denavit Hartenberg table of the arm
%  sigma    : [nx1] prismatic flag vector
%  Returns:
%  M : [nx] Inertia matrix

% Most of the functions are based on Robotics2Tools provided by MoSamArafat
% https://github.com/MoSamArafat/Robotics2Tools

% Made by Emanuele Giacomini (2020)

[n, ~] = size(dh_table);
end