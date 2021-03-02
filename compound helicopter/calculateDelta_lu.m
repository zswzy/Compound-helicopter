function [delta_l,delta_u] = CalculateDelta_lu(mu,delta_lFitted,delta_uFitted)
%[delta_l,delta_u] = CalculateDelta_lu(mu,delta_lFitted,delta_uFitted)
%   分段计算delta，需要提前知道mu的值
%   mu: 前进比
%   delta_lFitted： delta_l 对于 mu 的拟合模型
%   delta_uFitted： delta_u 对于 mu 的拟合模型
%   输出[delta_l, delta_u]的值

if mu < 0.2705
    delta_l = delta_lFitted(mu);
    delta_u = delta_uFitted(mu);
else
    delta_l = 0.44*exp(-(mu-0.2705)/(0.3962-0.2705));
    delta_u = 0.11*exp(-(mu-0.2705)/(0.3077-0.2705));
end