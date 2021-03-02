function [result_p_S2,result_q_S2] = Calculate_pq_S2()
%计算pq_S2
%   此处显示详细说明
global p q r 
global L_H2B2
global L_S2H2


omega       = [p;q;r];
%omega_dot   = [p_dot;q_dot;r_dot];
omega_S2    = L_S2H2*L_H2B2*omega;
%omega_S2_dot= L_S2H2*L_H2B2*omega_dot;
result_p_S2        = omega_S2(1);
result_q_S2 = omega_S2(2);
end