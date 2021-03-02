function [result_u_H1H1,result_v_H1H1,result_w_H1H1] = Calculate_uvw_H1H1()
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
global u v w p q r x_H1 y_H1 z_H1
global L_H1B1

u_H1B1 = u+z_H1*q - y_H1*r;
v_H1B1 = v+x_H1*r - z_H1*p;
w_H1B1 = w+y_H1*p - x_H1*q;
V_H1B1 = [u_H1B1;v_H1B1;w_H1B1];

V_H1H1 = L_H1B1 * V_H1B1;

u_H1H1 = V_H1H1(1);
v_H1H1 = V_H1H1(2);
w_H1H1 = V_H1H1(3);

result_u_H1H1 = u_H1H1;
result_v_H1H1 = v_H1H1;
result_w_H1H1 = w_H1H1;

end

