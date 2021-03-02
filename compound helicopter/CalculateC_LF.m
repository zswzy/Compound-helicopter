function result = CalculateC_LF(alpha,beta)
%计算机身的滚转力矩系数C_LF
%   alpha:机身迎角
%   beta：机身侧滑角
if (deg2rad(25) < abs(beta) && abs(beta) <= pi/2)
    result = -614.797*sin(beta) - beta/abs(beta)*(-47.7213*cos(4*beta) ...
        - 290.504*cos(beta)^3 + 735.507*cos(beta)^4 - 669.266);
elseif (deg2rad(10) < abs(beta) && abs(beta) <= deg2rad(25))
    result = - beta/abs(beta)*(455.707*cos(beta)^4 - 428.639);
else
    result = 0;
end
end

