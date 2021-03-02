function result = CalculateC_ZF(alpha,beta)
%机身升力系数C_ZF
%   alpha:机身迎角
%   beta：机身侧滑角
result = -35.3999*sin(beta) - 71.8019*sin(2*beta) + 8.04823*sin(4*beta) - 0.98*1e-12;
end


