function [result_u_H2H2,result_v_H2H2,result_w_H2H2] = Calculate_uvw_H2H2()
%UNTITLED4 此处显示有关此函数的摘要
%   此处显示详细说明
global u v w p q r x_H2 y_H2 z_H2
global L_H2B2

u_H2B2 = u+z_H2*q - y_H2*r;
v_H2B2 = v+x_H2*r - z_H2*p;
w_H2B2 = w+y_H2*p - x_H2*q;
V_H2B2 = [u_H2B2;v_H2B2;w_H2B2];

V_H2H2 = L_H2B2 * V_H2B2;

u_H2H2 = V_H2H2(1);
v_H2H2 = V_H2H2(2);
w_H2H2 = V_H2H2(3);

result_u_H2H2 = u_H2H2;
result_v_H2H2 = v_H2H2;
result_w_H2H2 = w_H2H2;

end