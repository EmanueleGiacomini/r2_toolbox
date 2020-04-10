function [tsf, critical_idx] = kr_scaling_factor(a, b, Cmin, Cmax)
%KR_SCALING_FACTOR Returns the best scaling factor for the most critical joint
%  a : [nx1] non constrained joint command
%  b : [nx1] exceeding commands factor
%  Cmin : [nx1] joint minimum command constraint
%  Cmax : [nx1] joint maximum command constraint
[n, ~] = size(a);
Smin(1:n) = 0;
Smax(1:n) = 0;
for i=1:n
    Smin(i) = (Cmin(i) - b(i)) / a(i);
    Smax(i) = (Cmax(i) - b(i)) / a(i);
    if Smin(i) > Smax(i)
        % swap i-th smin and smax
        temp = Smax(i);
        Smax(i) = Smin(i);
        Smin(i) = temp;
    end
end
smin = min(Smin);
[smax, critical_idx] = max(Smax);

if smin > smax || smax < 0 || smin > 1 
    tsf = 0;
else
    tsf = smax;
end
end

