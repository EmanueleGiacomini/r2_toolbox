function r = h2r(H)
%H2R Extract the Rotation matrix given an homogeneous transformation
%  H : [4x4] Homogeneous Transformation Matrix
%  Returns:
%  r : [3x3] Rotation matrix
r = H(1:3, 1:3);
end

