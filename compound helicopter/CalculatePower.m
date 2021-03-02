function [PW_MR1,PW_MR2,PW_PR,PW_Total] = CalculatePower(States)
%计算全机功率[PW_MR1,PW_MR2,PW_PR,PW_Total] = CalculatePower(States),W
%   States: a vector including all states variables and control variables
% States(1): theta_0
% States(2): theta_diff
% States(3): A
% States(4): B
% States(5): theta_PR
% States(6): v0_PR
% States(7): v_1
% States(8): v_2
% States(9): a_01
% States(10): a_11
% States(11): b_11
% States(12): a_02
% States(13): a_12
% States(14): b_12
% States(15): phi
% 只有这么多了

% 冗余变量 在主程序中预设
% theta
% A_diff
% B_diff
% delta_e
% delta_r


% States(1:20)需要求解，剩余为主函数预设值。
% 其中delta_e, delta_r, A_diff, B_diff, theta 为冗余变量
% 剩下共计15个~15个方程

%% 全局变量
global h
global rho
global g
global N_b m_b z_diff R A Omega sigma_M theta_t omega_n I_beta a c_d c K_beta gamma_M epsilon e M_beta i_theta i_phi x_H1 y_H1 z_H1 x_H2 y_H2 z_H2
global R_PR Omega_PR sigma_PR theta_tPR  a_PR c_dPR x_PR y_PR z_PR K_PR I_beta_PR c_PR kappa
global GW Ixx Iyy Izz 
global x_F y_F z_F l_F s_F 
global alpha_0HS s_H s_e xi_H x_HS y_HS z_HS
global alpha_0VS x_VS y_VS z_VS s_r xi_V s_V
global L_H1B1 L_H2B2
global psi V p q r
global theta A_diff B_diff delta_e delta_r
global delta_lFitted delta_uFitted

%求旋翼诱导速度用
global u_H1H1 v_H1H1 w_H1H1 u_H2H2 v_H2H2 w_H2H2
global T_W1 T_W2 delta_l delta_u

%%求尾桨诱导速度用
global C_TPR lambda_0PR B_v_PR B_w_PR
%% 定义对应状态 States
theta_0=        States(1);
theta_diff =    States(2);
A_1=                 States(3) - A_diff;
A_2=                 States(3) + A_diff;
B_1=                 States(4) - B_diff;
B_2=                 States(4) + B_diff;
theta_PR =      States(5);
v0_PR=              States(6);
v_1=                States(7);
v_2=                States(8);
a_01=               States(9);
a_11=               States(10);
b_11=               States(11);
a_02=               States(12);
a_12=               States(13);
b_12=               States(14);
phi=                States(15);

TBE = [cos(theta)*cos(psi)          cos(theta)*sin(psi)         -sin(theta);
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)      sin(phi)*cos(theta) ;
    cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)  cos(phi)*cos(theta)];

velocity_body = TBE*[V;0;0];
u = velocity_body(1);
v = velocity_body(2);
w = velocity_body(3);

%% 主旋翼 X_MR1 Y_MR1 Z_MR1 L_MR1 M_MR1 N_MR1 X_MR2 Y_MR2 Z_MR2 L_MR2 M_MR2 N_MR2
%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% delta_l = 0;
% delta_u = 0;

 %%%%%%%%%%%%%%%%%%%%%%%%%%
 % 计算lambda
% 需要已知 u v w p q r v_1 v_2 delta_l,u

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

lambda_1 = (w_H1H1-(v_1+delta_l*v_2))/(Omega*R);
lambda_2 = (w_H2H2-(v_2+delta_u*v_1))/(Omega*R);

%%%%%%%%%%%%%%%%%%%%%%%%%%
% 计算beta
beta_S1 = asin(v_H1H1/sqrt(u_H1H1^2+v_H1H1^2));
beta_S2 = asin(v_H2H2/sqrt(u_H2H2^2+v_H2H2^2));
% 计算转换矩阵
L_S1H1 = [cos(beta_S1)     sin(beta_S1) 0;
         -sin(beta_S1)    cos(beta_S1)  0;
            0               0           1];
L_S2H2 = [cos(beta_S2)     sin(beta_S2) 0;
         -sin(beta_S2)    cos(beta_S2)  0;
            0               0           1];
        
omega_S1    = L_S1H1*L_H1B1*[p;q;r];
p_S1        = omega_S1(1);
q_S1 = omega_S1(2);

omega_S2    = L_S2H2*L_H2B2*[p;q;r];
p_S2        = omega_S2(1);
q_S2 = omega_S2(2);
        
 %%%%%%%%%%%%%%%%%%%%%%%%%%
 % W为在过渡桨毂-气流坐标系SW
% 最终得到 MR1Force，MR2Force，MR1Moment，MR2Moment：向量形式
% X_MR1，Y_MR1，Z_MR1，L_MR1，M_MR1，N_MR1：标量形式
% X_MR2，Y_MR2，Z_MR2，L_MR2，M_MR2，N_MR2：标量形式

% 用诱导速度计算旋翼拉力 I代表诱导速度 下文直接求解
T_W1_I = 2*rho*A*v_1*sqrt(u_H1H1^2+v_H1H1^2+(-w_H1H1+v_1+delta_l*v_2)^2);
T_W2_I = 2*rho*A*v_2*sqrt(u_H2H2^2+v_H2H2^2+(-w_H2H2+v_2+delta_u*v_1)^2);

Chi_1 = atan(-mu_1/lambda_1);
Chi_2 = atan(-mu_2/lambda_2);

K_1 = 15*pi/32*tan(Chi_1/2);
K_2 = 15*pi/32*tan(Chi_2/2);

% 不计一阶变化小量
a_dot_11 = 0; b_dot_11 = 0;
a_dot_12 = 0; b_dot_12 = 0;
a_dot_01 = 0; a_dot_02 = 0;
a_ddot_01 = 0; a_ddot_02 = 0;
w_dot = 0;
p_dot_S1 = 0; q_dot_S1 = 0;
p_dot_S2 = 0; q_dot_S2 = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%拉力
%魏宁
% T_W1 = N_b/4*rho*a*c*Omega^2*R^3*((theta_0-theta_diff)*(2/3+mu_1^2-epsilon*mu_1^2) + theta_t*(1/2-2/3*epsilon ...
%         +1/2*(1-epsilon)^2*mu_1^2) - B_1*mu_1*(1-epsilon^2) + lambda_1*(1-epsilon^2) + a_11*epsilon*mu_1*(1-epsilon)...
%         + mu_1/(2*Omega)*p_S1*(1-epsilon^2) + a_dot_01/Omega*(-2/3+epsilon) + b_dot_11*mu_1/(2*Omega)*(1-epsilon)^2)... 
%         - N_b*(a_ddot_01*M_beta+m_b*g-m_b*(w_dot-u*q+p*v));

%张强
T_W1=N_b/4*rho*a*c*Omega^2*R^3*((theta_0-theta_diff)*(2/3+mu_1^2-epsilon*mu_1^2)+theta_t*(0.5+1/2*(1-epsilon^2)*mu_1^2)...
    -B_1*mu_1*(1-epsilon^2)+lambda_1*(1-epsilon^2)+a_11*epsilon*mu_1*(1-epsilon)+mu_1/2/Omega*p_S1*(1-epsilon^2)+a_dot_01/Omega*(-2/3+epsilon)...
    +b_dot_11*mu_1/2/Omega*(1-epsilon)^2);

%魏宁
% T_W2 = N_b/4*rho*a*c*Omega^2*R^3*((theta_0+theta_diff)*(2/3+mu_2^2-epsilon*mu_2^2) + theta_t*(1/2-2/3*epsilon ...
%         +1/2*(1-epsilon)^2*mu_2^2) - B_2*mu_2*(1-epsilon^2) + lambda_2*(1-epsilon^2) + a_12*epsilon*mu_2*(1-epsilon)...
%         + mu_2/(2*Omega)*p_S2*(1-epsilon^2) + a_dot_02/Omega*(-2/3+epsilon) + b_dot_12*mu_2/(2*Omega)*(1-epsilon)^2)... 
%         - N_b*(a_ddot_02*M_beta+m_b*g-m_b*(w_dot-u*q+p*v));

%张强
T_W2=N_b/4*rho*a*c*Omega^2*R^3*((theta_0+theta_diff)*(2/3+mu_2^2-epsilon*mu_2^2)+theta_t*(0.5+1/2*(1-epsilon^2)*mu_2^2)...
    -B_2*mu_2*(1-epsilon^2)+lambda_2*(1-epsilon^2)+a_12*epsilon*mu_2*(1-epsilon)+mu_2/2/Omega*p_S2*(1-epsilon^2)+a_dot_01/Omega*(-2/3+epsilon)...
    +b_dot_12*mu_2/2/Omega*(1-epsilon)^2);


 % 解算诱导速度
%  [vi,fval,exitflag,output] = fsolve(@InducesVelocity,[10,10]);
%  v_1 = vi(1); v_2 = vi(2);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%侧向力
%魏宁
% Y_W1 = N_b/4*rho*a*c*Omega^2*R^3*((theta_0-theta_diff)*(b_11*(2/3-1/2*epsilon+mu_1^2-epsilon*mu_1^2)-3/2*a_01*mu_1*(1-epsilon^2)+1/(3*Omega)*q_S1) ...
%         + theta_t*(1/2*b_11*(1+mu_1^2)*(1-epsilon)^2-a_01*mu_1*(1-3/2*epsilon)+1/Omega*q_S1*(1/4-1/3*epsilon)) ...
%         - A_1*((1/2*lambda_1+1/(8*Omega)*p_S1*mu_1)*(1-epsilon^2)+a_11*mu_1*(1/2+1/4*epsilon-3/4*epsilon^2)) ...
%         + B_1*(a_01*(1/3+mu_1^2-epsilon*mu_1^2)-b_11*mu_1*(1-1/4*epsilon-3/4*epsilon^2)-1/(8*Omega)*mu_1*q_S1*(1-epsilon^2)) ...
%         + a_01*(a_11*(1/3-1/2*epsilon-2*mu_1^2+2*epsilon*mu_1^2)-1/(3*Omega)*p_S1) ...
%         + a_11*(1/2*b_11*mu_1*(1-epsilon)+7/(8*Omega)*q_S1*mu_1*(1-epsilon^2)) + b_11*(5/8*mu_1*p_S1*(1-epsilon^2)) ...
%         + lambda_1*(b_11*(3/2-2*epsilon+1/2*epsilon^2)-3*a_01*mu_1*(1-epsilon)+1/Omega*q_S1*(1-epsilon^2)) ...
%         + (K_1*v_1+K_2*v_2*delta_l)/(Omega*R)*((theta_0-theta_diff)*(-1/3+1/2*epsilon)+theta_t*(-1/4+2/3*epsilon-1/2*epsilon^2)-(lambda_1+7/8*a_11*mu_1-1/8*B_1*mu_1)*(1-epsilon)^2) ... 
%         + 2*a_dot_01*a_dot_11/Omega^2*(-1/3+epsilon-epsilon^2) + a_dot_01/Omega*(A_1*(1/3-1/2*epsilon))...
%         + a_dot_11/Omega*((theta_0-theta_diff)*(1/3-1/2*epsilon)+theta_t*(1/4-2/3*epsilon+1/2*epsilon^2)-1/8*B_1*mu_1*(1-epsilon)^2) ...
%         - b_dot_11/(8*Omega)*(A_1*mu_1*(1-epsilon)^2) + 1/Omega*(lambda_1*a_dot_11*(1-epsilon)^2) + 2*a_dot_01/(3*Omega)*(K_1*v_1+K_2*v_2*delta_l)/(Omega*R)*(1-epsilon)^3 ...
%         + 2*b_11*a_dot_01/Omega*(-1/3+epsilon-epsilon^2) ...
%         + 1/(4*Omega)*mu_1*(4*a_01*a_dot_01+3*a_11*a_dot_11+b_11*b_dot_11)*(1-epsilon)^2 ...
%         + 2*a_dot_01/Omega^2*q_S1*(-1/3+1/2*epsilon) + 1/Omega*(b_11*a_dot_01 +a_01*b_dot_11)*(-1/3+1/2*epsilon) ...
%         + 1/(8*Omega)*mu_1*(4*a_01*a_dot_01 + a_11*a_dot_11 +3*b_11*b_dot_11)*(1-epsilon)^2);
%张强
Y_W1=N_b/4*rho*a*c*Omega^2*R^3*((theta_0-theta_diff)*(b_11*(2/3-1/2*epsilon+mu_1^2-epsilon*mu_1^2)-3/2*a_01*mu_1*(1-epsilon^2)+1/3/Omega*q_S1)...
    +theta_t*(b_11*(epsilon/3-1/2)+1/2*b_11*(1-epsilon^2)*mu_1^2-a_01*mu_1+1/Omega*q_S1*(1/4-1/3*epsilon))...
    -A_1*((1/2*lambda_1+1/8/Omega*p_S1*mu_1)*(1-epsilon^2)+a_11*mu_1*(1/2+1/4*epsilon-3/4*epsilon^2))...
    +B_1*(a_01*(1/3+mu_1^2-epsilon*mu_1^2)-b_11*mu_1*(1-1/4*epsilon-3/4*epsilon^2)-1/8/Omega*mu_1*q_S1*(1-epsilon^2))...
    +a_01*(a_11*(1/3-1/2*epsilon-2*mu_1^2+2*epsilon*mu_1^2)-1/3/Omega*p_S1)...
    +a_11*(1/2*b_11*mu_1*(1-epsilon)+7/8/Omega*mu_1*q_S1*(1-epsilon^2))+b_11*(5/8/Omega*mu_1*p_S1*(1-epsilon^2))...
    +lambda_1*(b_11*(3/2-2*epsilon+1/2*epsilon^2)-3*a_01*mu_1*(1-epsilon)+1/Omega*q_S1*(1-epsilon^2))...
    +(K_1*v_1+K_2*delta_l*v_2)/Omega/R*((theta_0-theta_diff)*(-1/3+1/2*epsilon)+theta_t*(-1/4+2/3*epsilon-1/2*epsilon^2)...
    -(lambda_1+7/8*a_11*mu_1-1/8*B_1*mu_1)*(1-epsilon)^2)...
    +2*a_dot_01*a_dot_11/Omega^2*(-1/3+epsilon-epsilon^2)+a_dot_01/Omega*(A_1*(1/3-1/2*epsilon))...
    +a_dot_11/Omega*((theta_0-theta_diff)*(1/3-1/2*epsilon)+theta_t*(1/4-2/3*epsilon+1/2*epsilon^2)-1/8*B_1*mu_1*(1-epsilon)^2)...
    -b_dot_11/8/Omega*(A_1*mu_1*(1-epsilon)^2)+1/Omega*(lambda_1*a_dot_11*(1-epsilon)^2)...
    +2*a_dot_01/3/Omega*(K_1*v_1+K_2*delta_l*v_2)/Omega/R*(1-epsilon)^3+2*b_11*a_dot_01/Omega*(-1/3+epsilon-epsilon^2)...
    +1/4/Omega*mu_1*(4*a_01*a_dot_01+3*a_11*a_dot_11+b_11*b_dot_11)*(1-epsilon)^2+...
    2*a_dot_01/Omega^2*q_S1*(-1/3+1/2*epsilon)+1/Omega*(b_11*a_dot_01+a_01*b_dot_11)*(-1/3+1/2*epsilon)...
    +1/8/Omega*mu_1*(4*a_01*a_dot_01+a_11*a_dot_11+3*b_11*b_dot_11)*(1-epsilon)^2);
%魏宁
% Y_W2 = N_b/4*rho*a*c*Omega^2*R^3*((theta_0+theta_diff)*(b_12*(2/3-1/2*epsilon+mu_2^2-epsilon*mu_2^2)-3/2*a_02*mu_2*(1-epsilon^2)+1/(3*Omega)*q_S2) ...
%         + theta_t*(1/2*b_12*(1+mu_2^2)*(1-epsilon)^2-a_02*mu_2*(1-3/2*epsilon)+1/Omega*q_S2*(1/4-1/3*epsilon)) ...
%         - A_2*((1/2*lambda_2+1/(8*Omega)*p_S2*mu_2)*(1-epsilon^2)+a_12*mu_2*(1/2+1/4*epsilon-3/4*epsilon^2)) ...
%         + B_2*(a_02*(1/3+mu_2^2-epsilon*mu_2^2)-b_12*mu_2*(1-1/4*epsilon-3/4*epsilon^2)-1/(8*Omega)*mu_2*q_S2*(1-epsilon^2)) ...
%         + a_02*(a_12*(1/3-1/2*epsilon-2*mu_2^2+2*epsilon*mu_2^2)-1/(3*Omega)*p_S2) ...
%         + a_12*(1/2*b_12*mu_2*(1-epsilon)+7/(8*Omega)*q_S2*mu_2*(1-epsilon^2)) + b_12*(5/8*mu_2*p_S2*(1-epsilon^2)) ...
%         + lambda_2*(b_12*(3/2-2*epsilon+1/2*epsilon^2)-3*a_02*mu_2*(1-epsilon)+1/Omega*q_S2*(1-epsilon^2)) ...
%         + (K_2*v_2+K_1*v_1*delta_u)/(Omega*R)*((theta_0+theta_diff)*(-1/3+1/2*epsilon)+theta_t*(-1/4+2/3*epsilon-1/2*epsilon^2)-(lambda_2+7/8*a_12*mu_2-1/8*B_2*mu_2)*(1-epsilon)^2) ... 
%         + 2*a_dot_02*a_dot_12/Omega^2*(-1/3+epsilon-epsilon^2) + a_dot_02/Omega*(A_2*(1/3-1/2*epsilon))...
%         + a_dot_12/Omega*((theta_0+theta_diff)*(1/3-1/2*epsilon)+theta_t*(1/4-2/3*epsilon+1/2*epsilon^2)-1/8*B_2*mu_2*(1-epsilon)^2) ...
%         - b_dot_12/(8*Omega)*(A_2*mu_2*(1-epsilon)^2) + 1/Omega*(lambda_2*a_dot_12*(1-epsilon)^2) + 2*a_dot_02/(3*Omega)*(K_2*v_2+K_1*v_1*delta_u)/(Omega*R)*(1-epsilon)^3 ...
%         + 2*b_12*a_dot_02/Omega*(-1/3+epsilon-epsilon^2) ...
%         + 1/(4*Omega)*mu_2*(4*a_02*a_dot_02+3*a_12*a_dot_12+b_12*b_dot_12)*(1-epsilon)^2 ...
%         + 2*a_dot_02/Omega^2*q_S2*(-1/3+1/2*epsilon) + 1/Omega*(b_12*a_dot_02 +a_02*b_dot_12)*(-1/3+1/2*epsilon) ...
%         + 1/(8*Omega)*mu_2*(4*a_02*a_dot_02 + a_12*a_dot_12 +3*b_12*b_dot_12)*(1-epsilon)^2);

%张强
Y_W2=N_b/4*rho*a*c*Omega^2*R^3*((theta_0+theta_diff)*(b_12*(2/3-1/2*epsilon+mu_2^2-epsilon*mu_2^2)-3/2*a_02*mu_2*(1-epsilon^2)+1/3/Omega*q_S2)...
    +theta_t*(b_12*(epsilon/3-1/2)+1/2*b_12*(1-epsilon^2)*mu_2^2-a_02*mu_2+1/Omega*q_S2*(1/4-1/3*epsilon))...
    -A_2*((1/2*lambda_2+1/8/Omega*p_S2*mu_2)*(1-epsilon^2)+a_12*mu_2*(1/2+1/4*epsilon-3/4*epsilon^2))...
    +B_2*(a_02*(1/3+mu_2^2-epsilon*mu_2^2)-b_12*mu_2*(1-1/4*epsilon-3/4*epsilon^2)-1/8/Omega*mu_2*q_S2*(1-epsilon^2))...
    +a_02*(a_12*(1/3-1/2*epsilon-2*mu_2^2+2*epsilon*mu_2^2)-1/3/Omega*p_S2)...
    +a_12*(1/2*b_12*mu_2*(1-epsilon)+7/8/Omega*mu_2*q_S2*(1-epsilon^2))+b_12*(5/8/Omega*mu_2*p_S2*(1-epsilon^2))...
    +lambda_2*(b_12*(3/2-2*epsilon+1/2*epsilon^2)-3*a_02*mu_2*(1-epsilon)+1/Omega*q_S2*(1-epsilon^2))...
    +(K_2*v_2+K_1*delta_u*v_1)/Omega/R*((theta_0+theta_diff)*(-1/3+1/2*epsilon)+theta_t*(-1/4+2/3*epsilon-1/2*epsilon^2)...
    -(lambda_2+7/8*a_12*mu_2-1/8*B_2*mu_2)*(1-epsilon)^2)...
    +2*a_dot_02*a_dot_12/Omega^2*(-1/3+epsilon-epsilon^2)+a_dot_02/Omega*(A_2*(1/3-1/2*epsilon))...
    +a_dot_12/Omega*((theta_0+theta_diff)*(1/3-1/2*epsilon)+theta_t*(1/4-2/3*epsilon+1/2*epsilon^2)-1/8*B_2*mu_2*(1-epsilon)^2)...
    -b_dot_12/8/Omega*(A_2*mu_2*(1-epsilon)^2)+1/Omega*(lambda_2*a_dot_12*(1-epsilon)^2)...
    +2*a_dot_02/3/Omega*(K_2*v_2+K_1*delta_u*v_1)/Omega/R*(1-epsilon)^3+2*b_12*a_dot_02/Omega*(-1/3+epsilon-epsilon^2)...
    +1/4/Omega*mu_2*(4*a_02*a_dot_02+3*a_12*a_dot_12+b_12*b_dot_12)*(1-epsilon)^2+...
    2*a_dot_02/Omega^2*q_S2*(-1/3+1/2*epsilon)+1/Omega*(b_12*a_dot_02+a_02*b_dot_12)*(-1/3+1/2*epsilon)...
    +1/8/Omega*mu_2*(4*a_02*a_dot_02+a_12*a_dot_12+3*b_12*b_dot_12)*(1-epsilon)^2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%前向力
%魏宁
% H_W1 = N_b/4*rho*a*c*Omega^2*R^3*((theta_0-theta_diff)*(a_11*(2/3-1/2*epsilon)-lambda_1*mu_1*(1-epsilon)-1/(3*Omega)*p_S1) ...
%         + theta_t*(1/2*(a_11-lambda_1*mu_1)*(1-epsilon)^2-1/Omega*p_S1*(1/4-1/3*epsilon)) ...
%         + A_1*(1/3*a_01-1/4*b_11*epsilon*mu_1*(1-epsilon)+1/(8*Omega)*q_S1*mu_1*(1-epsilon^2)) ...
%         + B_1*(1/2*lambda_1*(1-epsilon^2)-1/4*a_11*mu_1*(2-3*epsilon+epsilon^2)+3/(8*Omega)*p_S1*mu_1*(1-epsilon^2)) ...
%         - a_01*(b_11*(1/3-1/2*epsilon)-1/2*a_01*mu_1*(1-epsilon^2)+1/(3*Omega)*p_S1) ...
%         + a_11*(1/4*a_11*mu_1*(2-epsilon+epsilon^2)-1/(8*Omega)*p_S1*mu_1*(1-epsilon^2)) ...
%         + b_11*(1/4*b_11*epsilon*mu_1*(1-epsilon)-1/(8*Omega)*q_S1*mu_1*(1-epsilon^2)) ...
%         + lambda_1*(a_11*(3/2-2*epsilon+1/2*epsilon^2)-1/Omega*p_S1*mu_1*(1-epsilon^2)) + c_d/a*mu_1*(1-epsilon^2) ...
%         + (K_1*v_1+K_2*v_2*delta_l)/(Omega*R)*(a_01*(1/3-1/2*epsilon)-1/8*mu_1*(1-epsilon)^2*(A_1-B_1)) + 2*a_dot_01*b_dot_11/Omega^2*(1/3-epsilon+epsilon^2) ...
%         + a_dot_11/(8*Omega)*A_1*mu_1*(1-epsilon)^2 + a_dot_01/Omega*(B_1*(-1/3+1/2*epsilon)+(theta_0-theta_diff)*mu_1/2*(1-epsilon)^2+theta_t*mu_1/3*(1-3*epsilon+3*epsilon^2)) ...
%         + b_dot_11/Omega*(3/8*B_1*mu_1*(1-epsilon)^2+(theta_0-theta_diff)*(-1/3+1/2*epsilon)+theta_t*(-1/4+2/3*epsilon-1/2*epsilon^2)) ...
%         + 1/Omega*((a_11*a_dot_01+a_01*a_dot_11)*(-1/3+1/2*epsilon)-1/8*mu_1*(a_11*b_dot_11+b_11*a_dot_11)*(1-epsilon)^2) ...
%         + 1/Omega*(2*a_11*a_dot_01*(-1/3+epsilon-epsilon^2)-lambda_1*b_dot_11*(1-epsilon)^2+2*a_dot_01/Omega*p_S1*(1/3-1/2*epsilon)));

%张强
H_W1=N_b/4*rho*a*c*Omega^2*R^3*((theta_0-theta_diff)*(a_11*(2/3-1/2*epsilon)-lambda_1*mu_1*(1-epsilon)-1/3/Omega*p_S1)...
    +theta_t*(-1/2*lambda_1*mu_1*(1-epsilon)^2-a_11*(1/3*epsilon-1/2)-1/Omega*p_S1*(1/4-1/3*epsilon))...
    +A_1*(1/3*a_01-1/4*b_11*epsilon*mu_1*(1-epsilon)+1/8/Omega*q_S1*mu_1*(1-epsilon^2))...
    +B_1*(1/2*lambda_1*(1-epsilon^2)-1/4*a_11*mu_1*(2-3*epsilon+epsilon^2)+3/8/Omega*p_S1*mu_1*(1-epsilon^2))...
    -a_01*(b_11*(1/3-1/2*epsilon)-1/2*a_01*mu_1*(1-epsilon^2)+1/3/Omega*p_S1)...
    +a_11*(1/4*a_11*mu_1*(2-epsilon-epsilon^2)-1/8/Omega*p_S1*mu_1*(1-epsilon^2))...
    +b_11*(1/4*b_11*epsilon*mu_1*(1-epsilon)-1/8/Omega*q_S1*mu_1*(1-epsilon^2))...
    +lambda_1*(a_11*(3/2-2*epsilon+1/2*epsilon^2)-1/Omega*p_S1*(1-epsilon^2))+c_d/a*mu_1*(1-epsilon^2)...
    +(K_1*v_1+K_2*delta_l*v_2)/Omega/R*(a_01*(1/3-1/2*epsilon)-1/8*mu_1*(1-epsilon)^2*(A_1-b_11))...
    +2*a_dot_01*b_dot_11/Omega^2*(1/3-epsilon+epsilon^2)+a_dot_11/8/Omega*A_1*mu_1*(1-epsilon)^2+...
    a_dot_01/Omega*(B_1*(-1/3+1/2*epsilon)+(theta_0-theta_diff)*mu_1/2*(1-epsilon)^2+theta_t*mu_1/3*(1-3*epsilon+3*epsilon^2))...
    +b_dot_11/Omega*(3/8*B_1*mu_1*(1-epsilon)^2+(theta_0-theta_diff)*(-1/3+1/2*epsilon)+theta_t*(-1/4+2/3*epsilon-1/2*epsilon^2))...
    +1/Omega*((a_11*a_dot_01+a_01*a_dot_11)*(-1/3+1/2*epsilon)-1/8*mu_1*(a_11*b_dot_11+b_11*a_dot_11)*(1-epsilon)^2)...
    +1/Omega*(2*a_11*a_dot_01*(-1/3+epsilon-epsilon^2)-lambda_1*b_dot_11*(1-epsilon)^2+2*a_dot_01/Omega*p_S1*(1/3-1/2*epsilon)));
%魏宁
% H_W2 = N_b/4*rho*a*c*Omega^2*R^3*((theta_0+theta_diff)*(a_12*(2/3-1/2*epsilon)-lambda_2*mu_2*(1-epsilon)-1/(3*Omega)*p_S2) ...
%         + theta_t*(1/2*(a_12-lambda_2*mu_2)*(1-epsilon)^2-1/Omega*p_S2*(1/4-1/3*epsilon)) ...
%         + A_2*(1/3*a_02-1/4*b_12*epsilon*mu_2*(1-epsilon)+1/(8*Omega)*q_S2*mu_2*(1-epsilon^2)) ...
%         + B_2*(1/2*lambda_2*(1-epsilon^2)-1/4*a_12*mu_2*(2-3*epsilon+epsilon^2)+3/(8*Omega)*p_S2*mu_2*(1-epsilon^2)) ...
%         - a_02*(b_12*(1/3-1/2*epsilon)-1/2*a_02*mu_2*(1-epsilon^2)+1/(3*Omega)*p_S2) ...
%         + a_12*(1/4*a_12*mu_2*(2-epsilon+epsilon^2)-1/(8*Omega)*p_S2*mu_2*(1-epsilon^2)) ...
%         + b_12*(1/4*b_12*epsilon*mu_2*(1-epsilon)-1/(8*Omega)*q_S2*mu_2*(1-epsilon^2)) ...
%         + lambda_2*(a_12*(3/2-2*epsilon+1/2*epsilon^2)-1/Omega*p_S2*mu_2*(1-epsilon^2)) + c_d/a*mu_2*(1-epsilon^2) ...
%         + (K_2*v_2+K_1*v_1*delta_u)/(Omega*R)*(a_02*(1/3-1/2*epsilon)-1/8*mu_2*(1-epsilon)^2*(A_2-B_2)) + 2*a_dot_02*b_dot_12/Omega^2*(1/3-epsilon+epsilon^2) ...
%         + a_dot_12/(8*Omega)*A_2*mu_2*(1-epsilon)^2 + a_dot_02/Omega*(B_2*(-1/3+1/2*epsilon)+(theta_0+theta_diff)*mu_2/2*(1-epsilon)^2+theta_t*mu_2/3*(1-3*epsilon+3*epsilon^2)) ...
%         + b_dot_12/Omega*(3/8*B_2*mu_2*(1-epsilon)^2+(theta_0+theta_diff)*(-1/3+1/2*epsilon)+theta_t*(-1/4+2/3*epsilon-1/2*epsilon^2)) ...
%         + 1/Omega*((a_12*a_dot_02+a_02*a_dot_12)*(-1/3+1/2*epsilon)-1/8*mu_2*(a_12*b_dot_12+b_12*a_dot_12)*(1-epsilon)^2) ...
%         + 1/Omega*(2*a_12*a_dot_02*(-1/3+epsilon-epsilon^2)-lambda_2*b_dot_12*(1-epsilon)^2+2*a_dot_02/Omega*p_S2*(1/3-1/2*epsilon)));
%张强
H_W2=N_b/4*rho*a*c*Omega^2*R^3*((theta_0+theta_diff)*(a_12*(2/3-1/2*epsilon)-lambda_2*mu_2*(1-epsilon)-1/3/Omega*p_S2)...
    +theta_t*(-1/2*lambda_2*mu_2*(1-epsilon)^2-a_12*(1/3*epsilon-1/2)-1/Omega*p_S2*(1/4-1/3*epsilon))...
    +A_2*(1/3*a_02-1/4*b_12*epsilon*mu_2*(1-epsilon)+1/8/Omega*q_S2*mu_2*(1-epsilon^2))...
    +B_2*(1/2*lambda_2*(1-epsilon^2)-1/4*a_12*mu_2*(2-3*epsilon+epsilon^2)+3/8/Omega*p_S2*mu_2*(1-epsilon^2))...
    -a_02*(b_12*(1/3-1/2*epsilon)-1/2*a_02*mu_2*(1-epsilon^2)+1/3/Omega*p_S2)...
    +a_12*(1/4*a_12*mu_2*(2-epsilon-epsilon^2)-1/8/Omega*p_S2*mu_2*(1-epsilon^2))...
    +b_12*(1/4*b_12*epsilon*mu_2*(1-epsilon)-1/8/Omega*q_S2*mu_2*(1-epsilon^2))...
    +lambda_2*(a_12*(3/2-2*epsilon+1/2*epsilon^2)-1/Omega*p_S2*(1-epsilon^2))+c_d/a*mu_2*(1-epsilon^2)...
    +(K_2*v_2+K_1*delta_u*v_1)/Omega/R*(a_02*(1/3-1/2*epsilon)-1/8*mu_2*(1-epsilon)^2*(A_2-b_12))...
    +2*a_dot_02*b_dot_12/Omega^2*(1/3-epsilon+epsilon^2)+a_dot_12/8/Omega*A_2*mu_2*(1-epsilon)^2+...
    a_dot_02/Omega*(B_2*(-1/3+1/2*epsilon)+(theta_0+theta_diff)*mu_2/2*(1-epsilon)^2+theta_t*mu_2/3*(1-3*epsilon+3*epsilon^2))...
    +b_dot_12/Omega*(3/8*B_2*mu_2*(1-epsilon)^2+(theta_0+theta_diff)*(-1/3+1/2*epsilon)+theta_t*(-1/4+2/3*epsilon-1/2*epsilon^2))...
    +1/Omega*((a_12*a_dot_02+a_02*a_dot_12)*(-1/3+1/2*epsilon)-1/8*mu_2*(a_12*b_dot_12+b_12*a_dot_12)*(1-epsilon)^2)...
    +1/Omega*(2*a_12*a_dot_02*(-1/3+epsilon-epsilon^2)-lambda_2*b_dot_12*(1-epsilon)^2+2*a_dot_02/Omega*p_S2*(1/3-1/2*epsilon)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%俯仰力矩
% 魏宁
% M_W1 = N_b/4*rho*a*c*Omega^2*R^4*epsilon*(A_1*(1/3+1/4*mu_1^2*(1-epsilon)) + 1/2*a_01*mu_1*(1-epsilon^2) ...
%         - 1/4*b_11*mu_1^2*(1-epsilon) - 1/(3*Omega)*q_S1 + ((K_1*v_1+K_2*v_2*delta_l)/(Omega*R)-b_11)*(1/3-1/2*epsilon) ...
%         - a_dot_11/Omega*(1/3-1/2*epsilon)) ...
%         + N_b/2*a_11*M_beta*epsilon*R*Omega^2;

% 张强
M_W1=N_b/4*rho*a*c*Omega^2*R^4*epsilon*(A_1*(1/3+1/4*mu_1^2*(1-epsilon))+0.5*a_01*mu_1*(1-epsilon^2)-1/4*b_11*mu_1^2*(1-epsilon)...
    -1/3/Omega*q_S1+(K_1*v_1+K_2*delta_l*v_2/Omega/R-b_11)*(1/3-1/2*epsilon)-a_dot_11/Omega*(1/3-1/2*epsilon))...
    +N_b/2*a_11*M_beta*epsilon*R*Omega^2;    % +N_b*K_beta*a_11/2

% 魏宁
% M_W2 = N_b/4*rho*a*c*Omega^2*R^4*epsilon*(A_2*(1/3+1/4*mu_2^2*(1-epsilon)) + 1/2*a_02*mu_2*(1-epsilon^2) ...
%         - 1/4*b_12*mu_2^2*(1-epsilon) - 1/(3*Omega)*q_S2 + ((K_2*v_2+K_1*v_1*delta_u)/(Omega*R)-b_12)*(1/3-1/2*epsilon) ...
%         - a_dot_12/Omega*(1/3-1/2*epsilon)) ...
%         + N_b/2*a_12*M_beta*epsilon*R*Omega^2;

% 张强
M_W2 = N_b/4*rho*a*c*Omega^2*R^4*epsilon*(A_2*(1/3+1/4*mu_2^2*(1-epsilon))+0.5*a_02*mu_2*(1-epsilon^2)-1/4*b_12*mu_2^2*(1-epsilon)...
    -1/3/Omega*q_S2+((K_2*v_2+K_1*delta_u*v_1)/Omega/R-b_12)*(1/3-1/2*epsilon)-a_dot_12/Omega*(1/3-1/2*epsilon))...
    +N_b/2*a_12*M_beta*epsilon*R*Omega^2; %+N_b*K_beta*a_12/2;    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%滚转力矩
% 魏宁
% L_W1 = N_b/4*rho*a*c*Omega^2*R^4*epsilon*((theta_0-theta_diff)*mu_1*(1-epsilon^2) + theta_t*mu_1*(2/3-epsilon) - B_1*(1/3+3/4*mu_1^2*(1-epsilon)) + lambda_1*mu_1*(1-epsilon) ...
%         - a_11*(1/3-1/2*epsilon) + 1/4*a_11*mu_1^2*(1-epsilon) + 1/(3*Omega)*p_S1 - a_dot_01*mu_1/(2*Omega)*(1-epsilon)^2 + b_dot_11/Omega*(1/3-1/2*epsilon)) ...
%         - N_b/2*b_11*M_beta*epsilon*R*Omega^2;

% 张强    
L_W1=N_b/4*rho*a*c*Omega^2*R^4*epsilon*((theta_0-theta_diff)*mu_1*(1-epsilon^2)+theta_t*mu_1/3-B_1*(1/3+3/4*mu_1^2*(1-epsilon))...
    +lambda_1*mu_1*(1-epsilon)-a_11*(1/3-1/2*epsilon)+1/4*a_11*mu_1^2*(1-epsilon)+1/3/Omega*p_S1-a_dot_01*mu_1/2/Omega*(1-epsilon)^2+...
    b_dot_11/Omega*(1/3-1/2*epsilon))-N_b/2*b_11*M_beta*epsilon*R*Omega^2;     % -N_b*K_beta*b_11/2

% 魏宁
% L_W2 = N_b/4*rho*a*c*Omega^2*R^4*epsilon*((theta_0+theta_diff)*mu_2*(1-epsilon^2) + theta_t*mu_2*(2/3-epsilon) - B_2*(1/3+3/4*mu_2^2*(1-epsilon)) + lambda_2*mu_2*(1-epsilon) ...
%         - a_12*(1/3-1/2*epsilon) + 1/4*a_12*mu_2^2*(1-epsilon) + 1/(3*Omega)*p_S2 - a_dot_02*mu_2/(2*Omega)*(1-epsilon)^2 + b_dot_12/Omega*(1/3-1/2*epsilon)) ...
%         - N_b/2*b_12*M_beta*epsilon*R*Omega^2;

% 张强 
L_W2=N_b/4*rho*a*c*Omega^2*R^4*epsilon*((theta_0+theta_diff)*mu_2*(1-epsilon^2)+theta_t*mu_2/3-B_2*(1/3+3/4*mu_2^2*(1-epsilon))...
    +lambda_2*mu_2*(1-epsilon)-a_12*(1/3-1/2*epsilon)+1/4*a_12*mu_2^2*(1-epsilon)+1/3/Omega*p_S2-a_dot_02*mu_2/2/Omega*(1-epsilon)^2+...
    b_dot_12/Omega*(1/3-1/2*epsilon))-N_b/2*b_12*M_beta*epsilon*R*Omega^2-N_b*K_beta*a_12/2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%偏航力矩
% 魏宁
% N_W1 = N_b/4*rho*a*c*Omega^2*R^4*((theta_0-theta_diff)*(2/3*lambda_1+1/2*a_11*mu_1*epsilon+1/(3*Omega)*p_S1*mu_1) ...
%         + theta_t*(lambda_1*(1/2-2/3*epsilon)+a_11*mu_1*epsilon*(1/3-1/2*epsilon)+1/Omega*p_S1*mu_1*(1/4-1/3*epsilon)) ...
%         + A_1*(b_11*(-1/4+1/3*epsilon-1/8*mu_1^2+1/8*epsilon^2*mu_1^2)+1/3*a_01*mu_1-1/(4*Omega)*q_S1) ...
%         + B_1*(a_11*(1/4-1/3*epsilon-1/8*mu_1^2+1/8*epsilon^2*mu_1^2)-1/2*lambda_1*mu_1*(1-epsilon^2)-1/(4*Omega)*p_S1) ...
%         + a_01*(b_11*mu_1*(-2/3+epsilon)+1/2*a_01*mu_1^2*(1-epsilon^2)-2/(3*Omega)*q_S1*mu_1) ...
%         + a_11*(a_11*(1/4-2/3*epsilon+1/2*epsilon^2+3/8*mu_1^2-3/8*epsilon^2*mu_1^2)-1/Omega*p_S1*(1/2-2/3*epsilon)) ...
%         + b_11*(b_11*(1/4-2/3*epsilon+1/2*epsilon^2+1/8*mu_1^2-1/8*epsilon^2*mu_1^2)+1/Omega*q_S1*(1/2-2/3*epsilon)) ...
%         + lambda_1*(lambda_1+a_11*mu_1)*(1-epsilon^2) - 1/(2*a)*c_d*(1+mu_1^2-epsilon^2*mu_1^2) + 1/(4*Omega^2)*(p_S1^2+q_S1^2) ...
%         + (K_1*v_1+K_2*v_2*delta_l)/(Omega*R)*(A_1*(1/4-1/3*epsilon)-b_11*(1/2-4/3*epsilon+epsilon^2)+a_01*mu_1*(2/3-epsilon)-1/Omega*q_S1*(1/2-2/3*epsilon)) ...
%         + ((K_1*v_1+K_2*v_2*delta_l)/(Omega*R))^2*(1/4-2/3*epsilon+1/2*epsilon^2)+1/Omega^2*(2*a_dot_01^2+a_dot_11^2+b_dot_11^2)*(1/4-2/3*epsilon+1/2*epsilon^2) ...
%         + a_dot_01/Omega*((theta_0-theta_diff)*(-1/2+2/3*epsilon)+theta_t*(-2/5+epsilon-2/3*epsilon^2)+B_1*mu_1*(1/3-1/2*epsilon)) ...
%         + a_dot_11/Omega*A_1*(-1/4+1/3*epsilon) - 2/(3*Omega)*lambda_1*a_dot_01*(2-3*epsilon) ...
%         + b_dot_11/Omega*((theta_0-theta_diff)*mu_1*(1/3-1/2*epsilon)+theta_t*mu_1*(1/4-2/3*epsilon+1/2*epsilon^2)+B_1*(-1/4+1/3*epsilon)) ...
%         + 2*a_dot_11/Omega*(K_1*v_1+K_2*v_2*delta_l)/(Omega*R)*(-1/4+2/3*epsilon-1/2*epsilon^2) ...
%         + 2/Omega*(b_11*a_dot_11-a_11*b_dot_11)*(1/4-2/3*epsilon+1/2*epsilon^2)+2/Omega*mu_1*(a_01*a_dot_11+a_11*a_dot_01)*(-1/3+1/2*epsilon) ...
%         + 2/Omega^2*(b_dot_11*p_S1+a_dot_11*q_S1)*(1/4-1/3*epsilon));

% 张强 
N_W1=N_b/4*rho*a*c*Omega^2*R^4*((theta_0-theta_diff)*(2/3*lambda_1+1/2*a_11*mu_1*epsilon+1/3/Omega*p_S1*mu_1)...
    +theta_t*(1/2*lambda_1+1/3*a_11*epsilon*mu_1+1/Omega*p_S1*mu_1*(1/4-1/3*epsilon))...
    +A_1*(b_11*(-1/4+1/3*epsilon-1/8*mu_1^2+1/8*epsilon^2*mu_1^2)+1/3*a_01*mu_1-1/4/Omega*q_S1)...
    +B_1*(a_11*(1/4-1/3*epsilon-1/8*mu_1^2+1/8*epsilon^2*mu_1^2)-1/2*lambda_1*mu_1*(1-epsilon^2)-1/4/Omega*p_S1)...
    +a_01*(b_11*mu_1*(-2/3+epsilon)+1/2*a_01*mu_1^2*(1-epsilon^2)-2/3/Omega*q_S1*mu_1)...
    +a_11*(a_11*(1/4-2/3*epsilon+1/2*epsilon^2+3/8*mu_1^2-3/8*epsilon^2*mu_1^2)-1/Omega*p_S1*(1/2-2/3*epsilon))...
    +b_11*(b_11*(1/4-2/3*epsilon+1/2*epsilon^2+1/8*mu_1^2-1/8*epsilon^2*mu_1^2)+1/Omega*q_S1*(1/2-2/3*epsilon))...
    +lambda_1*(lambda_1+a_11*mu_1)*(1-epsilon^2)-1/2*a*c_d*(1+mu_1^2-epsilon^2*mu_1^2)+1/4/Omega*(p_S1^2+q_S1^2)...
    +(K_1*v_1+K_2*delta_l*v_2)/Omega/R*(A_1*(1/4-1/3*epsilon)-b_11*(1/2-4/3*epsilon+epsilon^2)...
    +a_01*mu_1*(2/3-epsilon)-1/Omega*q_S1*(1/2-2/3*epsilon))...
    +((K_1*v_1+K_2*delta_l*v_2)/Omega/R)^2*(1/4-2/3*epsilon+1/2*epsilon^2)...
    +1/Omega^2*(2*a_dot_01^2+a_dot_11^2+b_dot_11^2)*(1/4-2/3*epsilon+1/2*epsilon^2)...
    +a_dot_01/Omega*((theta_0-theta_diff)*(-1/2+2/3*epsilon)+theta_t*(-2/5+epsilon-2/3*epsilon^2)+B_1*mu_1*(1/3-1/2*epsilon))...
    +a_dot_11/Omega*A_1*(-1/4+1/3*epsilon)-2/3/Omega*lambda_1*a_dot_01*(2-3*epsilon)...
    +b_dot_11/Omega*((theta_0-theta_diff)*mu_1*(1/3-1/2*epsilon)+theta_t*mu_1*(1/4-2/3*epsilon+1/2*epsilon^2)+B_1*(-1/4+1/3*epsilon))...
    +2*a_dot_11/Omega*(K_1*v_1+K_2*delta_l*v_2)/Omega/R*(-1/4+2/3*epsilon-1/2*epsilon^2)...
    +2/Omega*(b_11*a_dot_11-a_11*b_dot_11)*(1/4-2/3*epsilon+1/2*epsilon^2)...
    +2/Omega*mu_1*(a_01*a_dot_11+a_11*a_dot_01)*(-1/3+1/2*epsilon)...
    +2/Omega^2*(b_dot_11*p_S1+a_dot_11*q_S1)*(1/4-1/3*epsilon));


% 魏宁
% N_W2 = N_b/4*rho*a*c*Omega^2*R^4*((theta_0+theta_diff)*(2/3*lambda_2+1/2*a_12*mu_2*epsilon+1/(3*Omega)*p_S2*mu_2) ...
%         + theta_t*(lambda_2*(1/2-2/3*epsilon)+a_12*mu_2*epsilon*(1/3-1/2*epsilon)+1/Omega*p_S2*mu_2*(1/4-1/3*epsilon)) ...
%         + A_2*(b_12*(-1/4+1/3*epsilon-1/8*mu_2^2+1/8*epsilon^2*mu_2^2)+1/3*a_02*mu_2-1/(4*Omega)*q_S2) ...
%         + B_2*(a_12*(1/4-1/3*epsilon-1/8*mu_2^2+1/8*epsilon^2*mu_2^2)-1/2*lambda_2*mu_2*(1-epsilon^2)-1/(4*Omega)*p_S2) ...
%         + a_02*(b_12*mu_2*(-2/3+epsilon)+1/2*a_02*mu_2^2*(1-epsilon^2)-2/(3*Omega)*q_S2*mu_2) ...
%         + a_12*(a_12*(1/4-2/3*epsilon+1/2*epsilon^2+3/8*mu_2^2-3/8*epsilon^2*mu_2^2)-1/Omega*p_S2*(1/2-2/3*epsilon)) ...
%         + b_12*(b_12*(1/4-2/3*epsilon+1/2*epsilon^2+1/8*mu_2^2-1/8*epsilon^2*mu_2^2)+1/Omega*q_S2*(1/2-2/3*epsilon)) ...
%         + lambda_2*(lambda_2+a_12*mu_2)*(1-epsilon^2) - 1/(2*a)*c_d*(1+mu_2^2-epsilon^2*mu_2^2) + 1/(4*Omega^2)*(p_S2^2+q_S2^2) ...
%         + (K_2*v_2+K_1*v_1*delta_u)/(Omega*R)*(A_2*(1/4-1/3*epsilon)-b_12*(1/2-4/3*epsilon+epsilon^2)+a_02*mu_2*(2/3-epsilon)-1/Omega*q_S2*(1/2-2/3*epsilon)) ...
%         + ((K_2*v_2+K_1*v_1*delta_u)/(Omega*R))^2*(1/4-2/3*epsilon+1/2*epsilon^2)+1/Omega^2*(2*a_dot_02^2+a_dot_12^2+b_dot_12^2)*(1/4-2/3*epsilon+1/2*epsilon^2) ...
%         + a_dot_02/Omega*((theta_0+theta_diff)*(-1/2+2/3*epsilon)+theta_t*(-2/5+epsilon-2/3*epsilon^2)+B_2*mu_2*(1/3-1/2*epsilon)) ...
%         + a_dot_12/Omega*A_2*(-1/4+1/3*epsilon) - 2/(3*Omega)*lambda_2*a_dot_02*(2-3*epsilon) ...
%         + b_dot_12/Omega*((theta_0+theta_diff)*mu_2*(1/3-1/2*epsilon)+theta_t*mu_2*(1/4-2/3*epsilon+1/2*epsilon^2)+B_2*(-1/4+1/3*epsilon)) ...
%         + 2*a_dot_12/Omega*(K_2*v_2+K_1*v_1*delta_u)/(Omega*R)*(-1/4+2/3*epsilon-1/2*epsilon^2) ...
%         + 2/Omega*(b_12*a_dot_12-a_12*b_dot_12)*(1/4-2/3*epsilon+1/2*epsilon^2)+2/Omega*mu_2*(a_02*a_dot_12+a_12*a_dot_02)*(-1/3+1/2*epsilon) ...
%         + 2/Omega^2*(b_dot_12*p_S2+a_dot_12*q_S2)*(1/4-1/3*epsilon));

% 张强 
N_W2=N_b/4*rho*a*c*Omega^2*R^4*((theta_0+theta_diff)*(2/3*lambda_2+1/2*a_12*mu_2*epsilon+1/3/Omega*p_S2*mu_2)...
    +theta_t*(1/2*lambda_2+1/3*a_12*epsilon*mu_2+1/Omega*p_S2*mu_2*(1/4-1/3*epsilon))...
    +A_2*(b_12*(-1/4+1/3*epsilon-1/8*mu_2^2+1/8*epsilon^2*mu_2^2)+1/3*a_02*mu_2-1/4/Omega*q_S2)...
    +B_2*(a_12*(1/4-1/3*epsilon-1/8*mu_2^2+1/8*epsilon^2*mu_2^2)-1/2*lambda_2*mu_2*(1-epsilon^2)-1/4/Omega*p_S2)...
    +a_02*(b_12*mu_2*(-2/3+epsilon)+1/2*a_02*mu_2^2*(1-epsilon^2)-2/3/Omega*q_S2*mu_2)...
    +a_12*(a_12*(1/4-2/3*epsilon+1/2*epsilon^2+3/8*mu_2^2-3/8*epsilon^2*mu_2^2)-1/Omega*p_S2*(1/2-2/3*epsilon))...
    +b_12*(b_12*(1/4-2/3*epsilon+1/2*epsilon^2+1/8*mu_2^2-1/8*epsilon^2*mu_2^2)+1/Omega*q_S2*(1/2-2/3*epsilon))...
    +lambda_2*(lambda_2+a_12*mu_2)*(1-epsilon^2)-1/2*a*c_d*(1+mu_2^2-epsilon^2*mu_2^2)+1/4/Omega*(p_S2^2+q_S2^2)...
    +(K_2*v_2+K_1*delta_u*v_1)/Omega/R*(A_2*(1/4-1/3*epsilon)-b_12*(1/2-4/3*epsilon+epsilon^2)...
    +a_02*mu_2*(2/3-epsilon)-1/Omega*q_S2*(1/2-2/3*epsilon))...
    +((K_2*v_2+K_1*delta_u*v_1)/Omega/R)^2*(1/4-2/3*epsilon+1/2*epsilon^2)...
    +1/Omega^2*(2*a_dot_02^2+a_dot_12^2+b_dot_12^2)*(1/4-2/3*epsilon+1/2*epsilon^2)...
    +a_dot_02/Omega*((theta_0+theta_diff)*(-1/2+2/3*epsilon)+theta_t*(-2/5+epsilon-2/3*epsilon^2)+B_2*mu_2*(1/3-1/2*epsilon))...
    +a_dot_12/Omega*A_2*(-1/4+1/3*epsilon)-2/3/Omega*lambda_2*a_dot_02*(2-3*epsilon)...
    +b_dot_12/Omega*((theta_0+theta_diff)*mu_2*(1/3-1/2*epsilon)+theta_t*mu_2*(1/4-2/3*epsilon+1/2*epsilon^2)+B_2*(-1/4+1/3*epsilon))...
    +2*a_dot_12/Omega*(K_2*v_2+K_1*delta_u*v_1)/Omega/R*(-1/4+2/3*epsilon-1/2*epsilon^2)...
    +2/Omega*(b_12*a_dot_12-a_12*b_dot_12)*(1/4-2/3*epsilon+1/2*epsilon^2)...
    +2/Omega*mu_2*(a_02*a_dot_12+a_12*a_dot_02)*(-1/3+1/2*epsilon)...
    +2/Omega^2*(b_dot_12*p_S2+a_dot_12*q_S2)*(1/4-1/3*epsilon));

% S1过渡气流坐标系
X_S1 = -H_W1;
Y_S1 = Y_W1;
Z_S1 = T_W1;
L_S1 = -L_W1;
M_S1 = M_W1;
N_S1 = -N_W1;

% S1左手旋翼气流坐标系
X_S2 = -H_W2;
Y_S2 = Y_W2;
Z_S2 = T_W2;
L_S2 = -L_W2;
M_S2 = M_W2;
N_S2 = -N_W2;

MR2Force_left_hand = L_H2B2'*L_S2H2'*[X_S2;Y_S2;Z_S2];
X_MR2_left_hand = MR2Force_left_hand(1);
Y_MR2_left_hand = MR2Force_left_hand(2);
Z_MR2_left_hand = MR2Force_left_hand(3);

MR2Moment_left_hand = L_H2B2'*L_S2H2'*[L_S2;M_S2;N_S2] + [y_H2*Z_MR2_left_hand-z_H2*Y_MR2_left_hand ; z_H2*X_MR2_left_hand-x_H2*Z_MR2_left_hand ; x_H2*Y_MR2_left_hand-y_H2*X_MR2_left_hand];
L_MR2_left_hand = MR2Moment_left_hand(1);
M_MR2_left_hand = MR2Moment_left_hand(2);
N_MR2_left_hand = MR2Moment_left_hand(3);

X_MR2 = X_MR2_left_hand;
Y_MR2 = -Y_MR2_left_hand;
Z_MR2 = Z_MR2_left_hand;
MR2Force = [X_MR2;Y_MR2;Z_MR2];

L_MR2 = -L_MR2_left_hand;
M_MR2 = M_MR2_left_hand;
N_MR2 = -N_MR2_left_hand;
MR2Moment = [L_MR2;M_MR2;N_MR2];

MR1Force = L_H1B1'*L_S1H1'*[X_S1;Y_S1;Z_S1];
X_MR1 = MR1Force(1);
Y_MR1 = MR1Force(2);
Z_MR1 = MR1Force(3);

MR1Moment = L_H1B1'*L_S1H1'*[L_S1;M_S1;N_S1] + [y_H1*Z_MR1-z_H1*Y_MR1 ; z_H1*X_MR1-x_H1*Z_MR1 ; x_H1*Y_MR1-y_H1*X_MR1];
L_MR1 = MR1Moment(1);
M_MR1 = MR1Moment(2);
N_MR1 = MR1Moment(3);        
 
%% 旋翼挥舞补充方程
% 旋翼挥舞方程，计算旋翼的锥度角a0，横纵向挥舞角a0,b0,控制量

% 下旋翼桨盘运动阻尼矩阵 D_1
D_1 = Omega* [gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon)            0           -gamma_M*mu_1/4*(1/3-epsilon+epsilon^2);
               0                            gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2)                             2;
               -gamma_M*mu_1*(1/3-epsilon+epsilon^2)              -2          gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2)];

% 上旋翼桨盘运动阻尼矩阵 D_2
D_2 = Omega* [gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon)            0           -gamma_M*mu_2/4*(1/3-epsilon+epsilon^2);
               0                            gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2)                             2;
               -gamma_M*mu_2*(1/3-epsilon+epsilon^2)              -2          gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2)];           
           
% 下桨盘刚度矩阵 mK_1
k_1 = zeros(3,3);
k_1(1,1) = 1 + K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
k_1(1,2) = -gamma_M*mu_1/4*(1/2*epsilon-epsilon^2);
k_1(1,3) = 0;
k_1(2,1) = -gamma_M*mu_1/2*(1/3-1/2*epsilon);
k_1(2,2) = K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
k_1(3,3)  = gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma_M*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
k_1(3,1) = 0;
k_1(3,2) = -gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma_M*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
k_1(3,3) = K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
mK_1 = Omega^2*k_1;

% 上桨盘刚度矩阵 mK_2
k_2 = zeros(3,3);
k_2(1,1) = 1 + K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
k_2(1,2) = -gamma_M*mu_2/4*(1/2*epsilon-epsilon^2);
k_2(1,3) = 0;
k_2(2,1) = -gamma_M*mu_2/2*(1/3-1/2*epsilon);
k_2(2,2) = K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
k_2(3,3)  = gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma_M*mu_2^2/8*(1/2-epsilon+1/2*epsilon^2);
k_2(3,1) = 0;
k_2(3,2) = -gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma_M*mu_2^2/8*(1/2-epsilon+1/2*epsilon^2);
k_2(3,3) = K_beta/(I_beta*Omega^2) + e*M_beta/I_beta;
mK_2 = Omega^2*k_2;

% 下旋翼外激励项 F1_1
f1_1 = zeros(3,4);
f1_1(1,1) = gamma_M/2*(1/4-1/3*epsilon)+gamma_M*mu_1^2/4*(1/2-epsilon+1/2*epsilon^2);
f1_1(1,2) = gamma_M/2*(1/5-1/4*epsilon)+gamma_M*mu_1^2/4*(1/3-1/2*epsilon);
f1_1(1,4) = -gamma_M*mu_1^2/2*(1/3-1/2*epsilon);
f1_1(2,3) = gamma_M/2*(1/4-1/3*epsilon) + gamma_M*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
f1_1(3,1) = -gamma_M*mu_1/2*(2/3-epsilon);
f1_1(3,2) = -gamma_M*mu_1/2*(1/2-2/3*epsilon);
f1_1(3,4) = gamma_M/2*(1/4-1/3*epsilon)+3*gamma_M*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
F1_1 = Omega^2*f1_1;

% 上旋翼外激励项 F1_2
f1_2 = zeros(3,4);
f1_2(1,1) = gamma_M/2*(1/4-1/3*epsilon)+gamma_M*mu_2^2/4*(1/2-epsilon+1/2*epsilon^2);
f1_2(1,2) = gamma_M/2*(1/5-1/4*epsilon)+gamma_M*mu_2^2/4*(1/3-1/2*epsilon);
f1_2(1,4) = -gamma_M*mu_2^2/2*(1/3-1/2*epsilon);
f1_2(2,3) = gamma_M/2*(1/4-1/3*epsilon) + gamma_M*mu_2^2/8*(1/2-epsilon+1/2*epsilon^2);
f1_2(3,1) = -gamma_M*mu_2/2*(2/3-epsilon);
f1_2(3,2) = -gamma_M*mu_2/2*(1/2-2/3*epsilon);
f1_2(3,4) = gamma_M/2*(1/4-1/3*epsilon)+3*gamma_M*mu_2^2/8*(1/2-epsilon+1/2*epsilon^2);
F1_2 = Omega^2*f1_2;

% 下旋翼外激励项 F2_1
f2_1 = zeros(3,4);
f2_1(1,1) = gamma_M*mu_1/(8*Omega)*(2/3-epsilon);
f2_1(2,1) = -2/Omega*(1+e*M_beta/I_beta);
f2_1(2,2) = -gamma_M/(2*Omega)*(1/4-1/3*epsilon);
f2_1(2,4) = -1/Omega^2;
f2_1(3,1) = -gamma_M/(2*Omega)*(1/4-1/3*epsilon);
f2_1(3,2) = 2/Omega*(1+e*M_beta/I_beta);
f2_1(3,3) = -1/Omega^2;
F2_1 = Omega^2*f2_1;

% 上旋翼外激励项 F2_2
f2_2 = zeros(3,4);
f2_2(1,1) = gamma_M*mu_2/(8*Omega)*(2/3-epsilon);
f2_2(2,1) = -2/Omega*(1+e*M_beta/I_beta);
f2_2(2,2) = -gamma_M/(2*Omega)*(1/4-1/3*epsilon);
f2_2(2,4) = -1/Omega^2;
f2_2(3,1) = -gamma_M/(2*Omega)*(1/4-1/3*epsilon);
f2_2(3,2) = 2/Omega*(1+e*M_beta/I_beta);
f2_2(3,3) = -1/Omega^2;
F2_2 = Omega^2*f2_2;

% 下旋翼外激励项 F3_1
F3_1 = Omega^2*[gamma_M/2*(1/3-1/2*epsilon); 0 ; -gamma_M*mu_1/2*(1/2-epsilon+1/2*epsilon^2)];

% 上旋翼外激励项 F3_2
F3_2 = Omega^2*[gamma_M/2*(1/3-1/2*epsilon); 0 ; -gamma_M*mu_2/2*(1/2-epsilon+1/2*epsilon^2)];

% 下旋翼外激励项 F4_1

F4_1 = Omega^2*[M_beta/I_beta/Omega^2*(w_dot-u*q+v*p-g); gamma_M/2*(K_1*v_1+K_2*v_2*delta_l)/(Omega*R)*(1/4-2/3*epsilon+1/2*epsilon^2); 0];

% 上旋翼外激励项 F4_2
F4_2 = Omega^2*[M_beta/I_beta/Omega^2*(w_dot-u*q+(-v)*(-p)-g); gamma_M/2*(K_2*v_2+K_1*v_1*delta_u)/(Omega*R)*(1/4-2/3*epsilon+1/2*epsilon^2); 0];

% 下旋翼合外激励 F1
F1 = F1_1*[theta_0-theta_diff; theta_t; A_1; B_1]+F2_1*[p_S1; q_S1; p_dot_S1; q_dot_S1] + F3_1*lambda_1 + F4_1;

% 上旋翼合外激励 F2
F2 = F1_2*[theta_0+theta_diff; theta_t; -A_2; B_2]+F2_2*[p_S2; q_S2; p_dot_S2; q_dot_S2] + F3_2*lambda_2 + F4_2;

flap_1 = mK_1*[a_01; a_11; b_11] - F1;
flap_2 = mK_2*[a_02; a_12; b_12] - F2;

% disp('mK_1');disp(mK_1);
% disp('F1');disp(F1);
% disp('mK_2');disp(mK_2);
% disp('F2');disp(F2);
% disp(F4_1);disp(F4_2)


%% 推进螺旋桨 X_PR Y_PR Z_PR L_PR M_PR N_PR
% 配平量  尾桨的根部安装角theta_PR v_0PR

chi1=atan(mu_1/(-lambda_1));
chi2=atan(mu_2/(-lambda_2));
k_1PR=1.299+0.671*chi1-1.172*chi1^2+0.351*chi1^3;
k_2PR=1.299+0.671*chi2-1.172*chi2^2+0.351*chi2^3;
% w_iPR=k_1PR*v_1+k_2PR*v_2;
w_iPR=-0.1*v_1-0.1*v_2;


% 推进桨气动中心在机体坐标系下速度
vol_PR=[u*sqrt(K_PR)+z_PR*q-y_PR*r;v*sqrt(K_PR)+x_PR*r-z_PR*p;w*sqrt(K_PR)+y_PR*p-x_PR*q+w_iPR];
u_PR=vol_PR(1);
v_PR=vol_PR(2);
w_PR=vol_PR(3);

% disp(vol_PR)

B_u_PR=u_PR/(Omega_PR*R_PR);
B_v_PR=v_PR/(Omega_PR*R_PR);
B_w_PR=w_PR/(Omega_PR*R_PR);

mu_PR=sqrt(B_v_PR^2+B_w_PR^2);
lambda_0PR=-B_u_PR;        %  lambda_0PR=B_u_PR;


%  推进桨挥舞锥度角，拉力系数，反扭矩系数
gamma_PR=rho*a_PR*c_PR*R_PR^4/I_beta_PR;
a0_PR=kappa*gamma_PR/2*(1/4*(1+mu_PR^2)*theta_PR+(1/5+1/6*mu_PR^2)*theta_tPR-1/3*(v0_PR-lambda_0PR));
C_TPR=a_PR*sigma_PR*((1/3+1/2*mu_PR^2)*theta_PR+1/4*(1+mu_PR^2)*theta_tPR-1/2*(v0_PR-lambda_0PR));
%C_TPR=a_PR*sigma_PR*((1/3+1/2*mu_PR^2)*theta_PR+1/4*(1+mu_PR^2)*theta_tPR-1/2*(v0_PR-lambda_0PR));

C_DPR=0.009+0.3*(6*C_TPR/a_PR/sigma_PR)^2;
m_kPR=1/4*sigma_PR*C_DPR*(1+mu_PR^2)+kappa*a_PR*sigma_PR*((1/3*theta_PR+1/4*theta_tPR)*(v0_PR-lambda_0PR)-1/2*(v0_PR-lambda_0PR)^2-1/4*mu_PR^2*a0_PR^2);

% [vi,~,~,~] = fsolve(@InducesVelocityP,[5]);
% v0_PR = vi;

% 推进桨诱导速度所导出的拉力 X_PR_I = X_PR v0_PR是无量纲的
C_TPR_I = 4*v0_PR*sqrt((v0_PR-lambda_0PR)^2+B_v_PR^2+B_w_PR^2);
X_PR_I = 1/2*rho*pi*R_PR^2*(R_PR*Omega_PR)^2*C_TPR_I;

% 推进桨力，力矩在机体轴系下的表达式
F_propeller=1/2*rho*pi*R_PR^2*(R_PR*Omega_PR)^2*[C_TPR;0;0];
X_PR=F_propeller(1);
Y_PR=F_propeller(2);
Z_PR=F_propeller(3);


M_propeller=[0;X_PR*z_PR;-X_PR*y_PR]+1/2*rho*pi*R_PR^5*Omega_PR^2*[-m_kPR;0;0];     % *[-m_kPR;0;0];
L_PR=M_propeller(1);
M_PR=M_propeller(2);
N_PR=M_propeller(3);

%disp(R_PR*Omega_PR)
%NUAA
% global X_PRFitted L_PRFitted
% X_PR = X_PRFitted(theta_PR);
% Y_PR = 0;
% Z_PR = 0;
% 
% 
% L_PR = L_PRFitted(theta_PR);
% M_PR = z_PR*X_PR;
% N_PR = -y_PR*X_PR;
% 
% X_PR_I = X_PR;
% v0_PR = 0;

%% 机身 X_F Y_F Z_F L_F M_F N_F
chi1=atan(mu_1/(-lambda_1));
chi2=atan(mu_2/(-lambda_2));
k_1F=1.299+0.671*chi1-1.172*chi1^2+0.351*chi1^3;
k_2F=1.299+0.671*chi2-1.172*chi2^2+0.351*chi2^3;
% w_iF=k_1F*v_1+k_2F*v_2;
% w_iF=-k_1F*v_1-k_2F*v_2;
w_iF=-0.8*v_1-0.8*v_2;

u_F = u + z_F*q - y_F*r;
v_F = v + x_F*r - z_F*p;
w_F = w + y_F*p - x_F*q + w_iF;

if u_F == 0
    if w_F == 0
        alpha_F = 0;
    else
        alpha_F = pi/2;
    end
else
    alpha_F = atan(w_F/u_F);
end


if (u_F == 0 && v_F == 0)
    beta_F = 0;
else
    beta_F = atan(v_F/sqrt(u_F^2+v_F^2+w_F^2));
end 

q_F = 1/2*rho*(u_F^2+v_F^2+w_F^2);

% 机身气动系数
xs_alpha_CLF=deg2rad([-10.0986,-7.93095,-6.0119,-4.01,-2.00857,-0.00619048,2.0781,4.08,5.99857,8.08381,10.0024,12.0048,14.0905]);
CLF=[-0.0291429,-0.0274286,-0.0231429,-0.0197143,-0.0171429,-0.0128571,-0.0111429,-0.00771429,-0.00428571,-8.57E-04,0.00257143,0.00685714,0.0111429];
if alpha_F < deg2rad(-10.0986) 
    C_LF = 2*-0.0291429;
elseif alpha_F > deg2rad(14.0905) 
    C_LF = 2*0.0111429;
else
C_LF=2*interp1(xs_alpha_CLF,CLF,alpha_F,'spline');
end

xs_alpha_CMF=deg2rad([-10.3333	-8.25	-6.25	-4.16667	-2.25	-0.25	1.83333	3.83333	5.75	7.83333	9.75	11.75	13.75]);
CMF=[-0.0062069	-0.00551724	-0.00448276	-0.0037931	-0.00275862	-0.00103448	-6.90E-04	6.90E-04	0.00172414	0.00275862	0.00344828	0.00448276	0.00517241];
if alpha_F < deg2rad(-10.3333) 
    C_MF = 2*-0.0062069;
elseif alpha_F > deg2rad(13.75) 
    C_MF = 2*0.00517241;
else
C_MF=2*interp1(xs_alpha_CMF,CMF,alpha_F,'spline');
end


xs_CLF=[0.012	0.00942857	0.00514286	0.00171429	-0.00342857	-0.00685714	-0.0102857	-0.0145714	-0.0188571	-0.0231429	-0.0274286];
CDF=[0.0242429	0.0233952	0.0217048	0.0216857	0.0224905	0.0233048	0.0249524	0.0265952	0.0282381	0.029881	0.0323571];
C_DF=2*interp1(xs_CLF,CDF,C_LF/2,'spline');

xs_beta_CYF=deg2rad([-14.9607	-9.95511	-4.85971	-2.48597	0	2.50281	5	10.0954	15.0842]);
CYF=[0.0376543	0.0216049	0.00876543	0.00487654	0	-0.00802469	-0.015	-0.0278395	-0.0407407];
if beta_F < deg2rad(-14.9607)
    C_YF = 2*0.0376543;
elseif beta_F > deg2rad(15.0842)
    C_YF = 2*-0.0407407;
else
C_YF=2*interp1(xs_beta_CYF,CYF,beta_F,'spline');
end

xs_beta_CNF=deg2rad([-14.9164	-9.92281	-4.83368	-2.34242	0	2.64564	5.03582	10.2482	15.1519]);
CNF=[0.0382407	0.0244349	0.0127673	0.00690287	0.00E+00	-0.00586439	-0.0128283	-0.0275504	-0.0445327];
if beta_F < deg2rad(-14.9164)
    C_NF = 2*0.0382407;
elseif beta_F > deg2rad(15.1519)
    C_NF = 2*-0.0445327;
else
C_NF=2*interp1(xs_beta_CNF,CNF,beta_F,'spline');
end


xs_beta_CLF=deg2rad([-14.2432	-9.25539	-4.26762	-1.72483	0.72016	3.37853	5.71683	10.9135	15.9013]);
CRF=[-8.44E-04	-4.79E-04	-1.15E-04	1.10E-04	2.49E-04	3.94E-04	5.31E-04	9.41E-04	0.00130562];
if beta_F < deg2rad(-14.2432)
    C_RF = 2*-8.44E-04;
elseif beta_F > deg2rad(15.9013)
    C_RF = 2*0.00130562;
else
C_RF=2*interp1(xs_beta_CLF,CRF,beta_F,'spline');
end

% disp(['q_F:' num2str(q_F) ])

% 受力计算
FuselageForce=[-q_F*pi*R^2*C_DF;q_F*pi*R^2*C_YF;-q_F*pi*R^2*C_LF];
X_F=FuselageForce(1);
Y_F=FuselageForce(2);
Z_F=FuselageForce(3);

M_fuselage=[q_F*pi*R^2*2*R*C_RF;q_F*pi*R^2*2*R*C_MF;q_F*pi*R^2*2*R*C_NF]+[y_F*Z_F-z_F*Y_F;z_F*X_F-x_F*Z_F;x_F*Y_F-y_F*X_F];
L_F=M_fuselage(1);
M_F=M_fuselage(2);
N_F=M_fuselage(3);

%% 水平尾翼  X_HS Y_HS Z_HS L_HS M_HS N_HS
% 配平量：delta_e
% 上下旋翼对平尾垂向速度影响因子
k_1HS = -0.1;
k_2HS = -0.1;
% 动压损失系数，大于0小于等于1，越小损失越大
K_HS = 1; 
% 主旋翼对平尾下洗干扰速度
w_iHS = k_1HS*v_1 + k_2HS*v_2;

u_HS = u + z_HS*q - y_HS*r;
v_HS = v + x_HS*r - z_HS*p;
w_HS = w + y_HS*p - x_HS*q + w_iHS;

% 动压
q_HS = 1/2*K_HS*rho*(u_HS^2+v_HS^2+w_HS^2);

%alpha_0F为机身对平尾下洗
alpha_0F = 0;
alpha_HS = alpha_0F + alpha_0HS + atan(w_HS/u_HS); 
beta_HS = asin(v_HS/sqrt(u_HS^2+v_HS^2+w_HS^2));

% 升降舵效率因子
k_e = sqrt(s_e/s_H)*cos(xi_H);
% 升降舵升力
% L_e = q_HS*s_H*C_XHFitted(alpha_HS,alpha_F)*delta_e*k_e;
L_e = q_HS*s_H*3.5*delta_e*k_e;

% 平尾气动力，力矩
C_LHS=3.5*alpha_HS;        % C_LHS=-3.5*alpha_HS;
C_DHS=0;

F_horizontalstabilizer=[cos(beta_HS)*cos(alpha_HS),-sin(beta_HS)*cos(alpha_HS),-sin(alpha_HS);
                                    sin(beta_HS),                       cos(beta_HS),                       0;
                                    cos(beta_HS)*sin(alpha_HS),-sin(beta_HS)*sin(alpha_HS),cos(alpha_HS)]*...
                            [-q_HS*s_H*C_DHS;   
                            0;  
                            -q_HS*s_H*C_LHS + L_e];
X_HS=F_horizontalstabilizer(1);
Y_HS=F_horizontalstabilizer(2);
Z_HS=F_horizontalstabilizer(3);
% disp(X_HS)

M_horizontalstabilizer=[y_HS*Z_HS-z_HS*Y_HS;z_HS*X_HS-x_HS*Z_HS;x_HS*Y_HS-y_HS*X_HS];
L_HS=M_horizontalstabilizer(1);
M_HS=M_horizontalstabilizer(2);
N_HS=M_horizontalstabilizer(3);

%% 垂直尾翼 X_VS Y_VS Z_VS L_VS M_VS N_VS
% 上下旋翼对垂尾垂向速度影响因子
k_1VS = -1;
k_2VS = -1;
% 动压损失系数，大于0小于等于1，越小损失越大
K_VS = 1; 
% 主旋翼对平尾下洗干扰速度
w_iVS = k_1VS*v_1 + k_2VS*v_2;

u_VS = u + z_VS*q - y_VS*r;
v_VS = v + x_VS*r - z_VS*p;
w_VS = w + y_VS*p - x_VS*q + w_iVS;

% 动压
q_VS = 1/2*K_VS*rho*(u_VS^2+v_VS^2+w_VS^2);

alpha_VS = alpha_0VS + atan(v_VS/u_VS); 
alpha_1VS = atan(v_VS/u_VS); 
beta_VS = asin(w_VS/sqrt(u_VS^2+v_VS^2+w_VS^2));

% 方向舵效率因子
k_r = sqrt(s_r/s_V)*cos(xi_V);
% 方向舵升力
L_r = -q_VS*s_V*3.5*delta_r*k_r;

% 垂尾气动力，力矩
C_LVS=3.5*alpha_VS;         % C_LVS=-3.5*beta_VS;
C_DVS=0;

F_verticalstabilizer=[cos(beta_VS)*cos(alpha_1VS),-sin(alpha_1VS),  -sin(beta_VS)*cos(alpha_1VS);
                            cos(beta_VS)*sin(alpha_1VS),    cos(alpha_1VS), -sin(beta_VS)*sin(alpha_1VS);
                               sin(beta_VS),                            0,                      cos(beta_VS)]*...
                            [-q_VS*s_V*C_DVS;
                             -q_VS*s_V*C_LVS + L_r;
                              0];
X_VS=F_verticalstabilizer(1);
Y_VS=F_verticalstabilizer(2);
Z_VS=F_verticalstabilizer(3);
% disp(X_VS)

M_verticalstabilizer=[y_VS*Z_VS-z_VS*Y_VS;z_VS*X_VS-x_VS*Z_VS;x_VS*Y_VS-y_VS*X_VS];
L_VS=M_verticalstabilizer(1);
M_VS=M_verticalstabilizer(2);
N_VS=M_verticalstabilizer(3);

%% 力、力矩合成 F_X F_Y F_Z L M N

F_X=X_MR1+X_MR2+X_F+X_HS+X_VS+X_PR-GW*g*sin(theta);
F_Y=Y_MR1+Y_MR2+Y_F+Y_HS+Y_VS+Y_PR+GW*g*cos(theta)*sin(phi);
F_Z=Z_MR1+Z_MR2+Z_F+Z_HS+Z_VS+Z_PR+GW*g*cos(theta)*cos(phi);

L=L_MR1+L_MR2+L_F+L_HS+L_VS+L_PR;
M=M_MR1+M_MR2+M_F+M_HS+M_VS+M_PR;
N=N_MR1+N_MR2+N_F+N_HS+N_VS+N_PR;
% disp(X_PR)

%% 功率计算 hpM马力 再算成W，要*746
% 1牛 = 1/4.445磅
% 1米 = 3.28英尺
% 1千克 = 2.205磅 = 1/14.5939slug
% rho : kg/m^3 = 1/14.5939/3.28^3 slug/ft^3 = 0.0019 slug/ft^3
f = 19.3/3.28^2;

hpM1 = T_W1_I/4.445*v_1*3.28 / 550 +(rho*0.0019)*f*(V*3.28)^3/1100 + (rho*0.0019)*(A*3.28^2)*sigma_M*(Omega*R*3.28)^3*c_d/8*(1+3*mu^2)/550;
hpM2 = T_W2_I/4.445*v_2*3.28 / 550 +(rho*0.0019)*f*(V*3.28)^3/1100 + (rho*0.0019)*(A*3.28^2)*sigma_M*(Omega*R*3.28)^3*c_d/8*(1+3*mu^2)/550;
hpT =  X_PR_I/4.445*v0_PR*3.28 / 550 + (rho*0.0019)*(A*3.28^2)*sigma_PR*(Omega_PR*R_PR*3.28)^2*c_d/8*(1+3*mu^2)/550;

%hpM1 = (T_W1_I/4.445)^2/(1000*(rho*0.0019)*(A*3.28^2)*(V*3.28)*(0.8))+(rho*0.0019)*f*(V*3.28)^3/1100 + (rho*0.0019)*(A*3.28^2)*sigma_M*(Omega*R*3.28)^2*c_d/8*(1+3*mu^2)/550;
%hpM2 = (T_W2_I/4.445)^2/(1000*(rho*0.0019)*(A*3.28^2)*(V*3.28)*(0.8))+(rho*0.0019)*f*(V*3.28)^3/1100 + (rho*0.0019)*(A*3.28^2)*sigma_M*(Omega*R*3.28)^2*c_d/8*(1+3*mu^2)/550;
%hpT = (X_PR_I/4.445)^2/(1000*(rho*0.0019)*pi*(R_PR*3.28)^2*(V*3.28)) + (rho*0.0019)*(A*3.28^2)*sigma_PR*(Omega_PR*R_PR*3.28)^2*c_d/8*(1+3*mu^2)/550;

PW_MR1 = hpM1*746;
PW_MR2 = hpM2*746;
PW_PR = hpT*746;
PW_Total = PW_MR1+PW_MR2+PW_PR;


%% 输出显示
global display
display = false;
if display == true
    component = {'MR1';'MR2';'F';'HS';'VS';'PR';'GW';'total'};
    X_Force = [X_MR1;X_MR2;X_F;X_HS;X_VS;X_PR;-GW*g*sin(theta);F_X];
    Y_Force = [Y_MR1;Y_MR2;Y_F;Y_HS;Y_VS;Y_PR;+GW*g*cos(theta)*sin(phi);F_Y];
    Z_Force = [Z_MR1 ;Z_MR2;Z_F;Z_HS;Z_VS;Z_PR;+GW*g*cos(theta)*cos(phi);F_Z];
    L_Moment = [L_MR1;L_MR2;L_F;L_HS;L_VS;L_PR;0;L];
    M_Moment = [M_MR1;M_MR2;M_F;M_HS;M_VS;M_PR;0;M];
    N_Moment = [N_MR1;N_MR2;N_F;N_HS;N_VS;N_PR;0;N];
    Induced_vs_Aero = [T_W1_I -T_W1; T_W2_I -T_W2;nan nan ;nan nan;nan nan;X_PR_I X_PR ;nan nan;nan nan];
    Power_kW = [PW_MR1/1000; PW_MR2/1000; nan;nan;nan;PW_PR/1000;nan;PW_Total/1000];
    Tvar = table(component,X_Force,Y_Force,Z_Force,L_Moment,M_Moment,N_Moment,Induced_vs_Aero,Power_kW);
%     varNames = {'FX','FY','FZ','L','M','N','Z_MR1','T_W1','T_W1_I','Z_MR2','T_W2','T_W2_I'};
%     varTypes = {'double','double','double','double','double','double','double','double','double','double','double','double'};
%     Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
%     Tvar(1,:) = {F_X,F_Y,F_Z,L,M,N,Z_MR1,T_W1,T_W1_I,Z_MR2,T_W2,T_W2_I};
    disp(Tvar)
end

end

