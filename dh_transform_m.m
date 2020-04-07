function H = dh_transform_m(dh_table, i, j)
%DH_TRANSFORM_M Returns the transformation matrix from two links, given DH Table
%  dh_table : [nx4] Denavit Hartenberg table of the arm
%  i        : [int] index for the source link
%  j        : [int] index for the destination link
%  Returns:
%  H : [4x4] Homogeneous Transformation Matrix from RFi to RFj
%
%  special cases:
%  i = 1, j = n : direct kinematics
%  i = k, j = k : link transform
%  % example for 2R direct kinematics extraction
%  dh_table = [0, 0, 1, q1;
%              0, 0, 1, q2];
%  direct_kin_matrix = dh_transform_m(dh_table, 1, 2)

% check for user input
if j < i
    error('Inverse transformation not yet implemented');
end
% no. of joints
[n, ~] = size(dh_table);
if i > n || i < 0 || j > n || j < 0
    error('i or j are out of range');
end

% initialize the transformation matrix
H = eye(4);
if i == j
    H = simplify(dh_step(dh_table(i, :)));
    return
end

for idx=i:j
    H = simplify(H*dh_step(dh_table(idx, :)));
end
end

