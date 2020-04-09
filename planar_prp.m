syms q1 q2 q3 dq1 dq2 dq3 real
syms m1 m2 m3 real
I1 = sym('I1', [3,3]);
I2 = sym('I2', [3,3]);
I3 = sym('I3', [3,3]);
inertia(:,:,1) = I1;
inertia(:,:,2) = I2;
inertia(:,:,3) = I3;
syms l1 l2 l3 real
syms k1 k2 real % offset from com of link 1
syms dc2 real % offset for Com of link 2

syms g0 real

g_vect = [0;0;-g0];

dht(1, :) = [pi/2, q1+k1, 0, 0];
dht(2, :) = [-pi/2, 0, l2, q2];
dht(3, :) = [0, q3 + k2, 0, 0];

offset(:, 1) = [0;0;-k1];
offset(:, 2) = [-l2 + dc2;0;0];
offset(:, 3) = [0;0;-k2];

dq = [dq1;dq2;dq3];
q = [q1;q2;q3];
sigma = [1;0;1];
mass = [m1;m2;m3];

% compute roc
roc(:, 1) = compute_roc(dht, offset, 1)
roc(:, 2) = compute_roc(dht, offset, 2)
roc(:, 3) = compute_roc(dht, offset, 3)

G = simplify(compute_gravity(q, mass, roc, g_vect))


%M = compute_sym_m(dht, dq, sigma, offset, mass, inertia)
%[c, S] = compute_christoffel(M, q, dq)

