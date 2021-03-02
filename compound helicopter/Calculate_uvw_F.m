function [result_u_F, result_v_F, result_w_F] = Calculate_uvw_F()
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
global x_F y_F z_F
global p q r
global u v w
result_u_F = u + z_F*q - y_F*r;
result_v_F = v + x_F*r - z_F*p;
result_w_F = w + y_F*p - x_F*q;
end

