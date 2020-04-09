function T = compute_roc(dh_table, offset, i)
%COMPUTE_ROC Compute the position of the CoM wrt RF0
%  dh_table : [nx4] Denavit Hartenberg table of the arm
%  offset   : [3xn] CoM offset vector (rci)
%  i        : [int] Index of the link to consider
%  Returns:
%  T : [3x1] : roc

dk = dh_transform_m(dh_table, 1, i);
lt = dh_transform_m(dh_table, i, i);

[rloc, tloc] = h2rt(lt);
if i==1
    % ric wrt index 1 is roc already
    ric = simplify(tloc + offset(:, i));
    T = ric;
    return
end
% use the offset in order to reach the CoM
ric = simplify(offset(:, i));
% use partial direct kinematic in order to reference Rf0
T = simplify(dk * [ric;1]);
T = simplify(T(1:3));

end
