function [result_p_S1,result_q_S1] = Calculate_pq_S1()
%计算p_S1
%   此处显示详细说明
global p q r 
global L_H1B1
global L_S1H1

omega       = [p;q;r];
%omega_dot   = [p_dot;q_dot;r_dot];
omega_S1    = L_S1H1*L_H1B1*omega;
%omega_S1_dot= L_S1H1*L_H1B1*omega_dot;
result_p_S1        = omega_S1(1);
result_q_S1 = omega_S1(2);
end

