% 计算mu
% 在已知 u v w p q r的情况下可以计算出mu
% 得到 V_H1H1及分量，V_H1B1及分量
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

u_H1H1 = V_H1H1(1);
v_H1H1 = V_H1H1(2);
w_H1H1 = V_H1H1(3);

u_H2H2 = V_H2H2(1);
v_H2H2 = V_H2H2(2);
w_H2H2 = V_H2H2(3);

mu_1 = sqrt(u_H1H1^2+v_H1H1^2)/(Omega*R);
mu_2 = sqrt(u_H2H2^2+v_H2H2^2)/(Omega*R);