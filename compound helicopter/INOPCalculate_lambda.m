function [result_lambda_1,result_lambda_2] = Calculate_lambda()
%计算lambda_1,lambda_2

global u v w p q r x_H1 y_H1 z_H1 x_H2 y_H2 z_H2
global L_H1B1 L_H2B2
global Omega R
global v_1 v_2
global delta_1 delta_2

u_H1B1 = u+z_H1*q - y_H1*r;
v_H1B1 = v+x_H1*r - z_H1*p;
w_H1B1 = w+y_H1*p - x_H1*q;

u_H2B2 = u+z_H2*q - y_H2*r;
v_H2B2 = v+x_H2*r - z_H2*p;
w_H2B2 = w+y_H2*p - x_H2*q;

V_H1B1 = [u_H1B1;v_H1B1;w_H1B1];
V_H2B2 = [u_H2B2;v_H2B2;w_H2B2];

V_H1H1 = L_H1B1 * V_H1B1;
V_H2H2 = L_H2B2 * V_H2B2;

w_H1H1 = V_H1H1(3);
w_H2H2 = V_H2H2(3);

result_lambda_1 = (w_H1H1-(v_1+delta_1*v_2))/(Omega*R);
result_lambda_2 = (w_H2H2-(v_2+delta_2*v_1))/(Omega*R);

end

