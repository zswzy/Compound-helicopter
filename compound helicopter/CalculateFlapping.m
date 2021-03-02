% 旋翼挥舞方程，计算旋翼的锥度角a0，横纵向挥舞角a0,b0,控制量

% 下旋翼桨盘运动阻尼矩阵 D_1
D_1 = Omega* [gamma/2*(1/4-2/3*epsilon+1/2*epsilon)            0           -gamma*mu_1/4*(1/3-epsilon+epsilon^2);
               0                            gamma/2*(1/4-2/3*epsilon+1/2*epsilon^2)                             2;
               -gamma*mu_1*(1/3-epsilon+epsilon^2)              -2          gamma/2*(1/4-2/3*epsilon+1/2*epsilon^2)];

% 上旋翼桨盘运动阻尼矩阵 D_2
D_2 = Omega* [gamma/2*(1/4-2/3*epsilon+1/2*epsilon)            0           -gamma*mu_2/4*(1/3-epsilon+epsilon^2);
               0                            gamma/2*(1/4-2/3*epsilon+1/2*epsilon^2)                             2;
               -gamma*mu_2*(1/3-epsilon+epsilon^2)              -2          gamma/2*(1/4-2/3*epsilon+1/2*epsilon^2)];           
           
% 下桨盘刚度矩阵 K_1
k_1 = zeros(3,3);
k_1(1,1) = 1 + K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
k_1(1,2) = -gamma*mu_1/4*(1/2*epsilon-epsilon^2);
k_1(1,3) = 0;
k_1(2,1) = -gamma*mu_1/2*(1/3-1/2*epsilon);
k_1(2,2) = K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
k_1(3,3)  = gamma/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
k_1(3,1) = 0;
k_1(3,2) = -gamma/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
k_1(3,3) = K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
K_1 = Omega^2*k_1;

% 上桨盘刚度矩阵 K_2
k_2 = zeros(3,3);
k_2(1,1) = 1 + K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
k_2(1,2) = -gamma*mu_2/4*(1/2*epsilon-epsilon^2);
k_2(1,3) = 0;
k_2(2,1) = -gamma*mu_2/2*(1/3-1/2*epsilon);
k_2(2,2) = K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
k_2(3,3)  = gamma/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma*mu_2^2/8*(1/2-epsilon+1/2*epsilon^2);
k_2(3,1) = 0;
k_2(3,2) = -gamma/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma*mu_2^2/8*(1/2-epsilon+1/2*epsilon^2);
k_2(3,3) = K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
K_2 = Omega^2*k_2;

% 下旋翼外激励项 F1_1
f1_1 = zeros(3,4);
f1_1(1,1) = gamma/2*(1/4-1/3*epsilon)+gamma*mu_1^2/4*(1/2-epsilon+1/2*epsilon^2);
f1_1(1,2) = gamma/2*(1/5-1/4*epsilon)+gamma*mu_1^2/4*(1/3-1/2*epsilon);
f1_1(1,4) = -gamma*mu_1^2/2*(1/3-1/2*epsilon);
f1_1(2,3) = gamma/2*(1/4-1/3*epsilon) + gamma*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
f1_1(3,1) = -gamma*mu_1/2*(2/3-epsilon);
f1_1(3,2) = -gamma*mu_1/2*(1/2-2/3*epsilon);
f1_1(3,4) = gamma/2*(1/4-1/3*epsilon)+3*gamma*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
F1_1 = Omega^2*f1_1;

% 上旋翼外激励项 F1_2
f1_2 = zeros(3,4);
f1_2(1,1) = gamma/2*(1/4-1/3*epsilon)+gamma*mu_2^2/4*(1/2-epsilon+1/2*epsilon^2);
f1_2(1,2) = gamma/2*(1/5-1/4*epsilon)+gamma*mu_2^2/4*(1/3-1/2*epsilon);
f1_2(1,4) = -gamma*mu_2^2/2*(1/3-1/2*epsilon);
f1_2(2,3) = gamma/2*(1/4-1/3*epsilon) + gamma*mu_2^2/8*(1/2-epsilon+1/2*epsilon^2);
f1_2(3,1) = -gamma*mu_2/2*(2/3-epsilon);
f1_2(3,2) = -gamma*mu_2/2*(1/2-2/3*epsilon);
f1_2(3,4) = gamma/2*(1/4-1/3*epsilon)+3*gamma*mu_2^2/8*(1/2-epsilon+1/2*epsilon^2);
F1_2 = Omega^2*f1_2;

% 下旋翼外激励项 F2_1
f2_1 = zeros(3,4);
f2_1(1,1) = gamma*mu_1/(8*Omega)*(2/3-epsilon);
f2_1(2,1) = -2/Omega*(1+e*M_beta/I_beta);
f2_1(2,2) = -gamma/(2*Omega)*(1/4-1/3*epsilon);
f2_1(2,4) = -1/Omega^2;
f2_1(3,1) = -gamma/(2*Omega)*(1/4-1/3*epsilon);
f2_1(3,2) = 2/Omega*(1+e*M_beta/I_beta);
f2_1(3,3) = -1/Omega^2;
F2_1 = Omega^2*f2_1;

% 上旋翼外激励项 F2_2
f2_2 = zeros(3,4);
f2_2(1,1) = gamma*mu_2/(8*Omega)*(2/3-epsilon);
f2_2(2,1) = -2/Omega*(1+e*M_beta/I_beta);
f2_2(2,2) = -gamma/(2*Omega)*(1/4-1/3*epsilon);
f2_2(2,4) = -1/Omega^2;
f2_2(3,1) = -gamma/(2*Omega)*(1/4-1/3*epsilon);
f2_2(3,2) = 2/Omega*(1+e*M_beta/I_beta);
f2_2(3,3) = -1/Omega^2;
F2_2 = Omega^2*f2_2;

% 下旋翼外激励项 F3_1
F3_1 = Omega^2*[gamma/2*(1/3-1/2*epsilon); 0 ; -gamma*mu_1/2*(1/2-epsilon+1/2*epsilon^2)];

% 上旋翼外激励项 F3_2
F3_2 = Omega^2*[gamma/2*(1/3-1/2*epsilon); 0 ; -gamma*mu_2/2*(1/2-epsilon+1/2*epsilon^2)];

% 下旋翼外激励项 F4_1
F4_1 = Omega^2*[M_beta/I_beta*(w_dot-u*q+v*p-g); gamma/2*(K_1*v_1+K_2*v_2*delta_l)/(Omega*R)*(1/4-2/3*epsilon+1/2*epsilon^2); 0];

% 上旋翼外激励项 F4_2
F4_2 = Omega^2*[M_beta/I_beta*(w_dot-u*q+(-v)*(-p)-g); gamma/2*(K_2*v_2+K_1*v_1*delta_u)/(Omega*R)*(1/4-2/3*epsilon+1/2*epsilon^2); 0];

% 下旋翼合外激励 F1
F1 = F1_1*[theta_0-theta_diff; theta_t; A_1; B_1]+F2_1*[p_S1; q_S1; p_dot_S1; q_dot_S1] + F3_1*lambda_1 + F4_1;

% 上旋翼合外激励 F2
F2 = F2_1*[theta_0+theta_diff; theta_t; -A_2; B_2]+F2_2*[p_S2; q_S2; p_dot_S2; q_dot_S2] + F3_2*lambda_2 + F4_2;