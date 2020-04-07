function [r, t] = h2rt(H)
%H2RT Extract the Rotation matrix and translation offset given an homogeneous transformation
%  H : [4x4] Homogeneous Transformation Matrix
%  Returns:
%  r : [3x3] Rotation matrix
%  t : [3x1] Translation offset vector
r = H(1:3, 1:3);
t = H(1:3, 4);
end

