% debug force
% force analysis for AllAerodynamics.m

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%hover, a_11,b_11,a_12,b12 = 0, A =B = 0
%配平状态
global V psi p q r 
h = 1000;
V = 0.1;
%预设值
psi=            0;   
p=              0;
q=              0;
r=              0;
global theta A_diff B_diff delta_e delta_r
% 冗余变量
theta = deg2rad(4);
A_diff = 0;
B_diff = 0;
delta_e = 0;
delta_r = 0;

UseTrimmedStates = true;
if UseTrimmedStates == true
    1;
else
    S.theta_0 = deg2rad(6.7914);% States(1): theta_0
    S.theta_diff =deg2rad( -2.5041);% States(2): theta_diff
    S.A =0.013814;% States(3): A
    S.B =0.95336;% States(4): B
    S.theta_PR =deg2rad(4.2142);% States(5): theta_PR
    S.v_0PR =-0.15963;% States(6): v0_PR
    S.v_1 =5.2985;% States(7): v_1
    S.v_2 =10.782;% States(8): v_2
    S.a_01 =-0.015032;% States(9): a_01
    S.a_11 =0.0051825;% States(10): a_11
    S.b_11 = 0.23196 ;% States(11): b_11
    S.a_02 =-0.022958;% States(12): a_02
    S.a_12 =-0.0020335;% States(13): a_12
    S.b_12 =0.23158;% States(14): b_12
    S.phi =deg2rad(-2.3438);% States(15): phi
end


% 输出显示
varNames = {'theta','phi','theta_0','theta_diff','theta_PR','v_0PR','v_1','v_2'};
varTypes = {'double','double','double','double','double','double','double','double'};
Tvar = table('Size',[1 length(varNames)],'VariableTypes',varTypes,'VariableNames',varNames);
Tvar(1,:) = {rad2deg(theta),rad2deg(S.phi),rad2deg(S.theta_0),rad2deg(S.theta_diff),rad2deg(S.theta_PR),S.v_0PR,S.v_1,S.v_2};
disp(Tvar)

% 计算测试
global display
display = true;
InitialStates = [S.theta_0 S.theta_diff S.A S.B S.theta_PR S.v_0PR S.v_1 S.v_2 S.a_01 S.a_11 S.b_11 S.a_02 S.a_12 S.b_12 S.phi];
F = AllAerodynamics(InitialStates);

%% 旋翼 悬停
global N_b m_b z_diff R A Omega sigma_M theta_t omega_n I_beta a c_d c K_beta gamma_M epsilon e M_beta i_theta i_phi x_H1 y_H1 z_H1 x_H2 y_H2 z_H2
global rho
theta_0 = 0.1;
theta_diff = 0;
mu_1 = 0; mu_2 = 0;
v_1 = 6; v_2 = 10;
lambda_1 = -v_1; lambda_2 = -v_2;
a_dot_01 = 0; b_dot_11 = 0; a_dot_02 = 0; b_dot_12 = 0;
A_1 = 0; B_1 = 0; A_2 = 0; B_2 = 0;
a_11 = 0; a_12 = 0;
p_S1 = 0; q_S1 = 0; p_S2 = 0; q_S2 = 0;

T_W1=N_b/4*rho*a*c*Omega^2*R^3*((theta_0-theta_diff)*(2/3+mu_1^2-epsilon*mu_1^2)+theta_t*(0.5+1/2*(1-epsilon^2)*mu_1^2)...
    -B_1*mu_1*(1-epsilon^2)+lambda_1*(1-epsilon^2)+a_11*epsilon*mu_1*(1-epsilon)+mu_1/2/Omega*p_S1*(1-epsilon^2)+a_dot_01/Omega*(-2/3+epsilon)...
    +b_dot_11*mu_1/2/Omega*(1-epsilon)^2);

T_W2=N_b/4*rho*a*c*Omega^2*R^3*((theta_0+theta_diff)*(2/3+mu_2^2-epsilon*mu_2^2)+theta_t*(0.5+1/2*(1-epsilon^2)*mu_2^2)...
    -B_2*mu_2*(1-epsilon^2)+lambda_2*(1-epsilon^2)+a_12*epsilon*mu_2*(1-epsilon)+mu_2/2/Omega*p_S2*(1-epsilon^2)+a_dot_01/Omega*(-2/3+epsilon)...
    +b_dot_12*mu_2/2/Omega*(1-epsilon)^2);
disp(T_W1)
disp(T_W2)

%%
theta_0 = deg2rad(5); theta_diff = 0;
a_11=0; b_11 = 0.01;
a_01 = 0.001;
mu_1 = 0;
A_1=0; B_1=0;
lambda_1 = -5;
v_1 = 5; v_2 = 10;
delta_l = 0.5;
p_S1=0; q_S1=0;
K_1=0; K_2=0;
a_dot_01=0; a_dot_11=0; b_dot_11=0;

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
    +1/8/Omega*mu_1*(4*a_01*a_dot_01+a_11*a_dot_11+3*b_11*b_dot_11)*(1-epsilon)^2)

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
    +1/Omega*(2*a_11*a_dot_01*(-1/3+epsilon-epsilon^2)-lambda_1*b_dot_11*(1-epsilon)^2+2*a_dot_01/Omega*p_S1*(1/3-1/2*epsilon)))