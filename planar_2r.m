syms q1 q2 dq1 dq2 real
syms m1 m2 real positive
syms l1 l2 d1 d2 real positive
syms ic1_zz ic2_zz real
syms g0 real 

% input parameters
dht = [0,0,l1,q1;0,0,l2,q2];
q = [q1;q2];
dq = [dq1;dq2];
sigma = [0;0];
offset(:, 1) = [d1-l1;0;0];
offset(:, 2) = [d2-l2;0;0];
mass = [m1;m2];
inertia(:,:,1) = [0,0,0;0,0,0;0,0,ic1_zz];
inertia(:,:,2) = [0,0,0;0,0,0;0,0,ic2_zz];
g_vect = [0;-g0;0];

roc(:, 1) = [0; d1*cos(q1); 0];
roc(:, 2) = [0; l1*sin(q1) + d2 * sin(q1+q2); 0];


M = compute_sym_m(dht, dq, sigma, offset, mass, inertia)
[chris, S] = compute_christoffel(M, q, dq) 
G = compute_gravity(q, mass, roc, g_vect)
