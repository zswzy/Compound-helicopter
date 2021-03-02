function result = CalculateC_MF(alpha,beta)
%计算机身的俯仰力矩系数C_MF
%   alpha:机身迎角
%   beta：机身侧滑角
result = 2.37925*alpha + 728.026*sin(2*alpha) + 426.76*sin(alpha)^2 ...
    + 348.072*cos(alpha) - 510.581*cos(beta)^3 + 56.111;
end

