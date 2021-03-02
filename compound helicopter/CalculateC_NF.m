function result = CalculateC_NF(alpha,beta)
%计算机身的偏航力矩系数C_NF
%   alpha:机身迎角
%   beta：机身侧滑角
if (deg2rad(20) < abs(beta) && abs(beta) <= pi/2)
    result = -220.0*sin(2*beta) - beta/abs(beta)*(671.0*cos(beta)^4-429.0);
else
    result = 278.133*sin(2*beta) - 422.644*sin(4*beta) - 1.83172 ;
end

