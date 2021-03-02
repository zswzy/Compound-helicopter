function result = CalculateC_XF(alpha,beta)
%机身阻力系数C_XF
%   alpha:机身迎角
%   beta：机身侧滑角
result = 90.0555*sin(alpha)^2 - 41.5604*cos(alpha) + 2.94684*cos(4*beta) - 103.14*cos(2*beta) ...
        - 0.53535*1e-6*beta^4 + 160.2049;
end

