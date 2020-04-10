function qpg = kr_task_pg(x, J, H, q)
%KR_TASK_PG Compute the commands q through the PG (Projected Gradient) method.
%  x  : [mx1] Task to be executed
%  J  : [mxn] Robot Jacobian
%  H  : [sym] Objective function to be optimized
%  q  : [nx1] q variables
%  Returns:
%  qdls : [nx1] PG solution.

[~, n] = size(J);
grad_q = gradient(H, q);
qpg = pinv(J) * x + (eye(n) - pinv(J) * J) * (grad_q * H);
end

