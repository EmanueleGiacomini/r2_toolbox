syms q1 q2 q3 dq1 dq2 dq3 real % joint and joint angvels
syms m1 m2 m3 real % masses
syms A C D E F real % offset values
syms ixx_1 iyy_1 izz_1 ixx_2 iyy_2 izz_2 ixx_3 iyy_3 izz_3 real
syms L1 L2 L3 real % distance dht params

% define input parameters
dht = [pi/2,L1,0, q1;0,0,L2,q2;0,0,L3,q3];
q = [q1;q2;q3];
dq = [dq1;dq2;dq3];
sigma = [0;0;0];

offset(:, 1) = [A;-F;0];
offset(:, 2) = [-C;0;0];
offset(:, 3) = [-D;0;E];

mass = [m1;m2;m3];

inertia(:,:,1) = [ixx_1,0,0;0,iyy_1,0;0,0,izz_1];
inertia(:,:,2) = [ixx_2,0,0;0,iyy_2,0;0,0,izz_2];
inertia(:,:,3) = [ixx_3,0,0;0,iyy_3,0;0,0,izz_3];

M = compute_sym_m(dht, dq, sigma, offset, mass, inertia)
[chris, S] = compute_christoffel(M, q, dq)