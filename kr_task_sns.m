function qsns = kr_task_sns(x, J, C)
%KR_TASK_SNS Compute the commands q through the SNS (Saturation of Null-Space) method
%  x : [mx1] Task to be executed
%  J : [mxn] Robot Jacobian
%  C : [nx1] constraints of the q commands.
%  Returns:
%  qsns : [nx1] SNS solution.
%
%  Please Care: We assume that constraints of commands C contains only
%  constraints in modulo wrt joint commands
%  ie: ||q|| <= C

% This function is based on the slides:
% 02_KinematicRedundancy_2.pdf page: 25
[m,n] = size(J);
% joint enable matrix (1 if enabled, 0 otherwise)
W = eye(n);
% saturated joints vector
qn(1:n, 1) = 0;
% current task scale factor
sf = 1;
% largest task scale factor so far
s_g = 0;

limit_ex = true;

while limit_ex == true
   limit_ex = false;
   q_exp = qn + pinv(J*W) * (x - J * qn);
   % check the joint velocity bounds
   % 
   saturated = [];
   for i = 1:n
       if (C(i) < abs(q_exp(i)))
           saturated = [saturated; 
               [i, q_exp(i) / abs(q_exp(i))]];
       end
   end
   if isempty(saturated)
       % No joints exceeded the limits
       disp("[SNS] Solution is in the bounds");
       break;
   end
   limit_ex = true;
   disp("[SNS] Solution exceeded some bounds");
   
   a = pinv(J * W) * x;
   b = q_exp - a;
   [sf, j] = kr_scaling_factor(a, b, -C, C);
   disp("[SNS] joint with max critical factor");
   disp(j);
   
   
   % check absolute scaling factor
   if sf > s_g
       s_g = sf;
       W_s = W;
       qns = qn;
   end
   
   % disable the most critic joint by forcing the saturated command
   W(j,j) = 0;
   if qn(j) > C(j)
       qn(j) = C(j);
   else
       qn(j) = -C(j);
   end
   % Check if task can b e accomplished with the remaining enabled joints
   if rank(J * W) < m
       disp("[SNS] Cannot complete task with the remaining enabled joints.");
       W = W_s;
       qn = qns;
       q_exp = qn + pinv(J * W) * (s_g * x - J * qn);
       limit_ex = false;
   end
end
qsns = q_exp;
end

