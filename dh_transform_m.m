function H = dh_transform_m(dh_table, i, j)
%DH_TRANSFORM_M Returns the transformation matrix from two links, given DH Table
%  dh_table : [nx4] Denavit Hartenberg table of the arm
%  i        : [int] index for the source link
%  j        : [int] index for the destination link
%  Returns:
%  H : [4x4] Homogeneous Transformation Matrix from RFi to RFj
%
%  % example for 2R direct kinematics extraction
%  dh_table = [...]
%  direct_kin_matrix = dh_transform_m(dh_table, 0, 2)

% no. of joints
[n, ~] = size(dh_table);
% initialize the transformation matrix
H = eye(4);

end

