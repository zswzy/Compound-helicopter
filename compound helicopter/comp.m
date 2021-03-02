function F = comp()
%UNTITLED3 此处显示有关此函数的摘要
%   此处显示详细说明
global T_W1_I T_W2_I u_H1H1 v_H1H1 w_H1H1 u_H2H2 v_H2H2 w_H2H2
global delta_lFitted delta_uFitted V Omega R delta_l delta_u
T_W1_I = 20000;
T_W2_I = 55000-T_W1_I;

mu = V/(Omega*R);

%%%%%%%%%%%%%%%%%%%%%%%%%%
% 计算delta
if mu < 0.2705
    delta_l = delta_lFitted(mu);
    delta_u = delta_uFitted(mu);
else
    delta_l = 0.44*exp(-(mu-0.2705)/(0.3962-0.2705));
    delta_u = 0.11*exp(-(mu-0.2705)/(0.3077-0.2705));
end
% hover
u_H1H1 = 0;
v_H1H1 = 0;
w_H1H1 = 0;
u_H2H2 = 0;
v_H2H2= 0;
w_H2H2 = 0;

[vi,fval,exitflag,output] = fsolve(@InducesVelocity,[10,10]);
disp(vi)
end



function F = InducesVelocity(vi)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
global u_H1H1 v_H1H1 w_H1H1 u_H2H2 v_H2H2 w_H2H2
global T_W1 T_W2
global rho A delta_l delta_u

v_1 = vi(1);
v_2 = vi(2);

F(1) = T_W1_I - 2*rho*A*v_1*sqrt(u_H1H1^2+v_H1H1^2+(-w_H1H1+v_1+delta_l*v_2)^2);
F(2) = T_W2_I - 2*rho*A*v_2*sqrt(u_H2H2^2+v_H2H2^2+(-w_H2H2+v_2+delta_u*v_1)^2);

end