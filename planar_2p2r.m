syms q1 q2 q3 q4 real
syms dq1 dq2 dq3 dq4 real
q = [q1;q2;q3;q4];
dq = [dq1;dq2;dq3;dq4];
syms m1 m2 m3 m4 real positive
mass = [m1;m2;m3;m4];
syms l1 l2 l3 l4 d1 d2 d3 d4 positive
I1 = sym('I1', [3,3]);
I2 = sym('I2', [3,3]);
I3 = sym('I3', [3,3]);
I4 = sym('I4', [3,3]);
inertia(:, :, 1) = I1;
inertia(:, :, 2) = I2;
inertia(:, :, 3) = I3;
inertia(:, :, 4) = I4;

offset(:, 1) = [0;0;-d1];
offset(:, 2) = [0;0;-d2];
offset(:, 3) = [-l3+d3;0;0];
offset(:, 4) = [-l4+d4;0;0];


dht(1, :) = [pi/2, q1, 0, 0];
dht(2, :) = [pi/2, q2, 0, pi/2];
dht(3, :) = [0, 0, l3, q3];
dht(4, :) = [0, 0, l4, q4];

sigma = [1;1;0;0];

M = compute_sym_m(dht, dq, sigma, offset, mass, inertia);
