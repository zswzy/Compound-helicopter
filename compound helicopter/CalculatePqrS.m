% 计算S1，S2系中的p q r 
% 已知：p r q 转换矩阵（betaS）

omega_S1    = L_S1H1*L_H1B1*[p;q;r];
p_S1 = omega_S1(1);
q_S1 = omega_S1(2);
r_S1 = omega_S1(3);

omega_S2    = L_S2H2*L_H2B2*[p;q;r];
p_S2 = omega_S2(1);
q_S2 = omega_S2(2);
r_S2 = omega_S2(3);