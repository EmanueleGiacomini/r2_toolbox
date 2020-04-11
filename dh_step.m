function H = dh_step(r)
%DH_STEP Compute a single step in the DH trasformation.
%  r : [1x4] Source row of the dh table
%  Returns:
%  H : [4x4] Homogeneous Transformation Matrix

% extracts the parameters from the row r
alpha = r(1);
d = r(2);
a = r(3);
theta = r(4);

H = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a * cos(theta);
     sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a * sin(theta);
     0, sin(alpha), cos(alpha), d;
     0, 0, 0, 1];
H = simplify(H);
end

