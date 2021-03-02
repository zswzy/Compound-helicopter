function [c,ceq] = nonlcon(x)
%NONLCON 此处显示有关此函数的摘要
%   此处显示详细说明
c       = zeros(size(x));
ceq     = double(x(2) && (x(2)-1));
end

