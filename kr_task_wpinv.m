function qwpinv = kr_task_wpinv(x, J, W)
%KR_TASK_WPINV Compute the commands q through the Weighted Pseudoinverse method
%  x : [mx1] Task to be executed
%  J : [mxn] Robot Jacobian
%  W : [nxn] Weight matrix 
%  Returns:
%  qwpinv : [nx1] WPinv solution.

Jw = inv(W) * J' * inv((J * inv(W) * J'));
qwpinv = Jw * x;
end

