function dc = compute_dcoeff(M, C, G, q, dq)
%COMPUTE_DCOEFF Compute Dynamic Coefficients of the model.
%  M  : [nxn] Inertia matrix
%  C  : [nx1] Coriolis/Centrifugal terms 
%  G  : [nx1] Gravity vector
%  q  : [nx1] Joint position vector
%  dq : [nx1] Joint velocity vector
%  Returns:
%  dc : [px1] Dynamic coefficients.

% Bear in mind that this method follows a custom euristic.

[n, ~] = size(M);

% coefficient vector
c_vect = []; 

% start from diagonal terms of M
for i = 1:n
    term = M(i,i);
    % TODO remove dependencies from q and dq
    term = subs(term, dq, zeros(n, 1));
    
    if ismember(term, c_vect)==0
        c_vect = [c_vect;term];
    end
    
    % TODO
end
dc = c_vect;
end

