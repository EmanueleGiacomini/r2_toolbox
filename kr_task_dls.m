function qdls = kr_task_dls(x, J, mu)
%KR_TASK_DLS Compute the commands q through the DLS (Dampened Least Squares) method.
%  x  : [mx1] Task to be executed
%  J  : [mxn] Robot Jacobian
%  mu : [double] variable dampening factor
%  Returns:
%  qdls : [nx1] DLS solution.

[m, ~] = size(J);
Jdls = J' * inv((J * J' + mu ^ 2 * eye(m)));
qdls = Jdls * x;
end

