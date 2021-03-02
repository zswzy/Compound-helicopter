function result = CalculateC_YF(alpha,beta)
%机身侧力系数C_YF
%   alpha:机身迎角
%   beta：机身侧滑角
result = 29.3616*sin(alpha) - 43.468*sin(2*alpha) - 81.8924*sin(alpha)^2 - 84.1469*cos(alpha) ...
        - 0.821406*1e-1*beta - 3.00102*sin(4*beta) + 0.323477*beta^2 + 85.3496;
end
