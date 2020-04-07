function M = compute_sym_m(dh_table, dq, sigma, offset, mass, inertia)
%COMPUTE_SYM_M Computes Inertia Matrix M through Lagrange-Euler method.
%  dh_table : [nx4] Denavit Hartenberg table of the arm
%  dq       : [nx1] joint velocities vector
%  sigma    : [nx1] prismatic flag vector
%  offset   : [nx1] CoM x-axis offset vector
%  mass     : [nx1] Link mass vector
%  inertia  : [3x3xn] Inertia tensor
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

zi = [0;0;1];
w0 = 0;
v0 = 0;

for i=1:n
    % computes the current link kinetic energy based on slides:
    % 04_LagrangianDynamics_2.pdf page: 5
    
    % rotation matrix (rot_i) and link offset (ri) based on d param
    % ri = dh_table(i, 2);
    % rot_i = h2r(dh_transform_m(dh_table, i, i));
    [rot_i, ri] = h2rt(dh_transform_m(dh_table, i, i));
    % CoM offset ( from RFi hence a subtraction of li from di )
    rci = [-dh_table(i, 3) + offset(i);0;0];
    sig = sigma(i);
    dqi = dq(i);
    % angvel i from RFi-1
    p_wi = w0 + (1-sig) * dqi * zi;
    
    % compute wi and vi (angvel and vel for link i)
    wi = simplify(rot_i' * p_wi);
    vi = simplify(rot_i' * (v0 + sig * dqi * zi + cross(p_wi, ri)));
    
    % compute vci (translational velocity of the CoM)
    vci = simplify(vi + cross(wi, rci));
    vci = simplify(vci)
    ic = inertia(:, :, i);
    % Kinetic energy of the i-th link
    ti = simplify(1/2 * (wi' * ic * wi + mass(i) * (vci' * vci)));
    
    % TODO(Emanuele): Use the kinetic energy to compute the intertia matrix
    
    v0 = vi;
    w0 = wi;
end
end