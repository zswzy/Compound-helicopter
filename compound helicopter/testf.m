function F = testf(States)
%UNTITLED5 此处显示有关此函数的摘要
%   此处显示详细说明
a = States(1);
b = States(2);

%a = 1;
F(1) = a^2+b^2-15;
F(2) = a-1;

end

