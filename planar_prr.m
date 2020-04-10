% Simple Planar Robot generator:
% Make changes in the "edit here" blocks.

% Inertia terms legend: 
% Iki_j -> intertia element of link k, row i and column j
% E.g. I43_3 -> zz inertia term of link 4

% ----------------- EDIT HERE ----------------- %
% Number of joints
n = 3;

% Type of joints (1 if prismatic, 0 if revolute)
sigma = [1;0;0];

% Gravity vector
syms g0 real
g_vect = [g0;0;0];
% --------------------------------------------- %

%syms('q', [1 n], real)
syms q [1 n] real
q = transpose(q);
syms dq [1 n] real
dq = transpose(dq);
syms ddq [1 n] real
ddq = transpose(ddq);
syms m [1 n] real
m = transpose(m);
syms l [1 n] real % Links length
l = transpose(l);
syms d [1 n] real % Joint-COM displacement
d = transpose(d);

% Inertia tensor
inertia = sym(zeros(3,3,n));
for i=1:n
    str = strcat('I',int2str(i));
    inertia(:,:,i) = sym(str, [3,3]);
end

% ----------------- EDIT HERE ----------------- %
% DH Table [ alpha, d, a, theta ]
dht(1, :) = [pi/2, q1, 0, 0];
dht(2, :) = [0, 0, l2, pi/2+q2];
dht(3, :) = [0, 0, l3, q3];

% COM offsets
offset(:, 1) = [0;0;-q1+d1];
offset(:, 2) = [-l2+d2;0;0];
offset(:, 3) = [-l3+d3;0;0];
% --------------------------------------------- %

% Compute roc
roc = sym(zeros(3, n));
for i=1:n
    roc(:, i) = compute_roc(dht, offset, i);
end

% Compute Robot Inertia Matrix
M = compute_sym_m(dht, dq, sigma, offset, m, inertia)
% Compute Christoffel and Skew-Symm
[c, S] = compute_christoffel(M, q, dq)
% Compute Gravity Term
G = simplify(compute_gravity(q, m, roc, g_vect))

