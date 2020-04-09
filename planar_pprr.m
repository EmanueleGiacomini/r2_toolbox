syms q1 q2 q3 q4 dq1 dq2 dq3 dq4 real
syms m1 m2 m3 m4 real
I1 = sym('I1', [3,3]);
I2 = sym('I2', [3,3]);
I3 = sym('I3', [3,3]);
I4 = sym('I4', [3,3]);
inertia(:,:,1) = I1;
inertia(:,:,2) = I2;
inertia(:,:,3) = I3;
inertia(:,:,4) = I4;
syms l1 l2 l3 l4 real
syms k real
syms d1 d2 d3 d4

syms g0 real

g_vect = [0;0;-g0];

% DH Table [ alpha, d, a, theta ]
dht(1, :) = [pi/2, q1, 0, 0];
dht(2, :) = [pi/2, q2, 0, pi/2];
dht(3, :) = [0, 0, l3, q3];
dht(4, :) = [0, 0, l4, q4];

offset(:, 1) = [0;0;-d1];
offset(:, 2) = [0;0;-d2];
offset(:, 3) = [-l3+d3;0;0];
offset(:, 4) = [-l4+d4;0;0];

dq = [dq1;dq2;dq3;dq4];
q = [q1;q2;q3;q4];
sigma = [1;1;0;0];
mass = [m1;m2;m3;m4];

% compute roc
%roc(:, 1) = compute_roc(dht, offset, 1);
%roc(:, 2) = compute_roc(dht, offset, 2);
%roc(:, 3) = compute_roc(dht, offset, 3);

%G = simplify(compute_gravity(q, mass, roc, g_vect));


M = compute_sym_m(dht, dq, sigma, offset, mass, inertia)
[c, S] = compute_christoffel(M, q, dq)

