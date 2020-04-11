function dparam = std_dparam(c, m, inertia)
%STD_DPARAM Compute the Standard Dynamic Parameters for a link
%  c       : [3x1] Local position of the CoM. (offset)
%  m       : [double] Mass of the link
%  inertia : [3x3] Inertia matrix of the link.
%  Returns:
%  dparam  : [1x10] Complete dynamic parameters for the link
mom1 = (m * c)';
% reduced inertia (only upper diag block)
rin = nonzeros(triu(inertia))';
dparam = [m, mom1, rin];
end

