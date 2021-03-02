function F = CoaxialRotor(States)
% 1.16
% 双旋翼动力学模型 下标1为下，2为上
% States(1) = v_01
% States(2) = v_02
% States(3) = theta_0
% States(4) = theta_1c
% States(5) = theta_1s
% 王泽远 1.15
% 1.16 单旋翼下标，单旋翼测试，参数传递。悬停状态测试。

%% 旋翼特性基本测试：模拟旋转塔，无线速度，角速度，横纵变距为0。给定总距，计算受力。
global rho
global Rotor b m_b R A Omega s theta_t I_beta a_0 delta c K_beta e M_beta gamma_s h_R1 h_R2 x_cg

global theta_01 theta_02

%% 运动状态，下旋翼
u_H1=0; v_H1=0; w_H1=0;
V = sqrt(u_H1^2+v_H1^2+w_H1^2);
u_dot=0; v_dot=0; w_dot=0;
p=0; q=0; r=0;
p_H1=0; q_H1=0;
p_dot=0; q_dot=0; r_dot=0;
p_dot_H1=0; q_dot_H1=0; 
%操纵变量 1c:纵向力 1s：横向力
theta_01 = States(3); theta_1c1=States(4); theta_1s1=States(5);
%诱导速度
v_01 = States(1); v_1c1 = 0; v_1s1 = 0;

%% 运动状态,上旋翼
u_H2=u_H1; v_H2=-v_H1; w_H2=w_H1;
u_dot=0; v_dot=0; w_dot=0;
p=0; q=0; r=0;
p_H2=-p_H1; q_H2=q_H1;
p_dot=0; q_dot=0; r_dot=0;
p_dot_H2=-p_dot_H1; q_dot_H2=q_dot_H1; 
%操纵变量 1c:纵向力 1s：横向力
theta_02 = States(3); theta_1c2=theta_1c1; theta_1s2=theta_1s1;
%诱导速度
v_02 = States(2); v_1c2 = 0; v_1s2 = 0;

%% 旋翼干扰模型
% delta_1_list = [1.35	1.31	1.2     1.05	0.9     0.8	0.7     0.57	0.44]';
% delta_2_list = [0.66	0.64	0.58	0.53	0.45	0.4	0.33	0.21	0.11]';
% mu_list = [0	0.0128	0.041	0.0808	0.1205	0.15	0.191	0.2282	0.2705]';
% delta_lFitted = fit(mu_list, delta_1_list, 'linearinterp');
% delta_uFitted = fit(mu_list, delta_2_list, 'linearinterp');
% mu = V/(Omega*R);
% if mu < 0.2705
%     delta_1 = delta_lFitted(mu);
%     delta_2 = delta_uFitted(mu);
% else
%     delta_1 = 0.44*exp(-(mu-0.2705)/(0.3962-0.2705));
%     delta_2 = 0.11*exp(-(mu-0.2705)/(0.3077-0.2705));
% end
% 
% v_01 = States(1) + 0.5*States(2);
% v_02 = States(2) + 0.2*States(1);

%% 下旋翼模型
beta_0p1 = 0; beta_1sp1 = 0; beta_1cp1 = 0; 
mu_x1 = u_H1/(Omega*R); mu_y1 = v_H1/(Omega*R); mu_z1 = w_H1/(Omega*R); mu_1 = sqrt(mu_x1^2+mu_y1^2);
q_bar_H1 = q_H1/Omega; p_bar_H1 = p_H1/Omega;
lambda_01 = v_01/(Omega*R); lambda_1c1 = v_1c1/(Omega*R); lambda_1s1 = v_1s1/(Omega*R); 
a_Hx1 = (u_dot-(q_dot+p*r)+(q^2+r^2))*cos(gamma_s)+(w_dot+(p^2+q^2)+(q_dot-p*r)*sin(gamma_s));
a_Hy1 = v_dot+(p_dot-q*r)-(r_dot+p*q);
a_Hz1 = (w_dot+(p^2+q^2)+(q_dot-p*r))*cos(gamma_s)-(u_dot-(q_dot+p*r)+(q^2+r^2))*sin(gamma_s);
m_bar_b = m_b/(rho*pi*R^3);
eta_x1 = a_Hx1/(Omega^2*R); eta_y1 = a_Hy1/(Omega^2*R); eta_z1 = a_Hz1/(Omega^2*R);

% 挥舞方程
n_beta = rho*c*a_0*R^4/(8*I_beta);
q_H1p1 = q_bar_H1/Omega^2; p_H1p1 = p_bar_H1/Omega^2;
lambda_beta = sqrt(1+K_beta/(I_beta*Omega^2));

hM01 = zeros(3,1);
hM01(1) = M_beta*R/I_beta*eta_z1+n_beta*(2/3*(1-e^3)*(2*(mu_z1-lambda_01)+((2*theta_1c1-lambda_1s1+p_bar_H1)*mu_y1+(2*theta_1s1-lambda_1c1+q_bar_H1)*mu_x1))+...
         (1-e^2)*(1+e^2+mu_1^2)*theta_01+(4/5-e+1/5*e^5+(2/3-e+1/3*e^3)*mu_1^2)*theta_t);

hM01(2) = 2*(p_bar_H1+q_H1p1/2)+n_beta*((1-e^2)*((1+e^2+1/2*mu_1^2+mu_y1^2)*theta_1c1+mu_x1*mu_y1*theta_1s1)+8/3*(1-e^3)*mu_y1*theta_01+...
        (2-8/3*e+2/3*e^4)*mu_y1*theta_t+2*(1-e^2)*(mu_z1-lambda_01)*mu_y1-(1-e^4)*(lambda_1c1-q_bar_H1));

hM01(3) = -2*(q_bar_H1-p_H1p1/2)+n_beta*((1-e^2)*((1+e^2+1/2*mu_1^2+mu_x1^2)*theta_1s1+mu_x1*mu_y1*theta_1c1)+8/3*(1-e^3)*mu_y1*theta_01+...
        (2-8/3*e+2/3*e^4)*mu_x1*theta_t+2*(1-e^2)*(mu_z1-lambda_01)*mu_x1-(1-e^4)*(lambda_1s1-p_bar_H1));

DM01 = zeros(3,3);
DM01(1,1) = lambda_beta^2;
DM01(1,2) = (M_beta*R/I_beta)*eta_x1/2;
DM01(1,3) = -(M_beta*R/I_beta)*eta_y1/2;
DM01(2,1) = (M_beta*R/I_beta)*eta_x1+4*mu_x1*n_beta/3 - 4*e^3*mu_x1*n_beta/3;
DM01(2,2) = mu_x1*mu_y1*n_beta*(1-e^2)+lambda_beta^2-1;
DM01(2,3) = n_beta*(1-e^2)*(2*e^2+mu_x1^2-mu_y1^2+2)/2;
DM01(3,1) = 4*mu_y1*n_beta*e^3/3 - (M_beta*R/I_beta)*eta_y1-4*mu_y1*n_beta/3;
DM01(3,2) = n_beta*(e^2-1)*(2*e^2-mu_x1^2+mu_y1^2+2)/2;
DM01(3,3) = mu_x1*mu_y1*n_beta*e^2+lambda_beta^2-mu_x1*mu_y1*n_beta-1;

flap1 = DM01\hM01;
beta_01 = flap1(1);
beta_1c1 = flap1(2);
beta_1s1 = flap1(3);

alpha_1c1 = q_bar_H1 - lambda_1c1 - beta_1cp1 - beta_1s1; alpha_1s1 = p_bar_H1 - lambda_1s1 - beta_1sp1 - beta_1c1; 
% 气动力系数
C_ZA01 = -1/3*beta_0p1*(1-e^3)+1/2*(1-e^2)*(mu_z1-lambda_01) + 1/4*(1-e^2)*(mu_x1*(alpha_1s1-beta_1c1)+mu_y1*(alpha_1c1+beta_1s1)) + ...
        1/2*(1-e^2)*(theta_1c1*mu_y1+theta_1s1*mu_x1) + (1/3*(1-e^3)+1/2*(1-e)*mu_1^2)*theta_01 + (1/4*(1+mu_1^2*(1-e)^2)-1/3*e)*theta_t;

C_ZA1c1 = 1/3*alpha_1c1*(1-e^3) + mu_y1*(1-e)*(mu_z1-lambda_01) - 1/2*(1-e^2)*(beta_0p1*mu_y1+beta_01*mu_x1) - 1/4*(1-e)*beta_1s1*(mu_x1^2-mu_y1^2) +...
        1/2*(1-e)*(theta_1s1-beta_1c1)*mu_x1*mu_y1 + (1/3*(1-e^3)+(1-e)*(1/4*mu_1^2+1/2*mu_y1^2))*theta_1c1 + mu_y1*(1-e^2)*theta_01 +...
        mu_y1*(2/3-e+1/3*e^3)*theta_t;

C_ZA1s1 = 1/3*alpha_1s1*(1-e^3) + mu_x1*(1-e)*(mu_z1-lambda_01) - 1/2*(1-e^2)*(beta_0p1*mu_x1+beta_01*mu_y1) - 1/4*(1-e)*beta_1c1*(mu_x1^2-mu_y1^2) +...
        1/2*(1-e)*(theta_1c1-beta_1s1)*mu_x1*mu_y1 + (1/3*(1-e^3)+(1-e)*(1/4*mu_1^2+1/2*mu_x1^2))*theta_1s1 + mu_x1*(1-e^2)*theta_01 +...
        mu_x1*(2/3-e+1/3*e^3)*theta_t;

C_YA1c1 = theta_1c1*(1/3*beta_0p1*(1-e^3)-1/2*(1-e^2)*(mu_z1-lambda_01)-1/8*(1-e^2)*((alpha_1s1-3*beta_1c1)*mu_x1 + (3*alpha_1c1+beta_1s1)*mu_y1) + 1/2*(1-e)*beta_01*mu_x1*mu_y1)+...
        theta_1s1*(-1/8*(1-e^2)*((alpha_1c1-beta_1s1)*mu_x1 + (alpha_1s1+beta_1c1)*mu_y1) + 1/4*(1-e)*beta_01*(mu_x1^2-mu_y1^2)) + ...
        theta_01*(-1/3*(1-e^3)*alpha_1c1+1/2*(1-e^2)*(beta_0p1*mu_y1+beta_01*mu_x1) - mu_y1*(1-e)*(mu_z1-lambda_01) + 1/4*(1-e)*beta_1s1*(mu_x1^2-mu_y1^2)+1/2*(1-e)*beta_1c1*mu_x1*mu_y1)+...
        theta_t*(-alpha_1c1*(1/4-1/3*e)+1/3*(1-3/2*e+1/2*e^3)*(beta_0p1*mu_y1+beta_01*mu_x1)-1/2*mu_y1*(mu_z1-lambda_01)*(1-e)^2+1/8*beta_1s1*(mu_x1^2-mu_y1^2)*(1-e)^2+1/4*beta_1c1*mu_x1*mu_y1*(1-e)^2)+...
        alpha_1c1*(2/3*(1-e^3)*beta_0p1-(mu_z1-lambda_01)*(1-e^2)+3/4*beta_1c1*mu_x1*(1-e^2)-1/4*beta_1s1*mu_y1*(1-e^2))+...
        1/4*alpha_1s1*(1-e^2)*(beta_1s1*mu_x1-beta_1c1*mu_y1) + beta_01*mu_x1*(-beta_0p1*(1-e^2)+2*(1-e)*(mu_z1-lambda_01)-3/2*beta_1c1*mu_x1*(1-e)+beta_1s1*mu_y1*(1-e))-...
        1/2*beta_01*beta_1c1*mu_y1^2*(1-e)+delta/a_0*(1-e^2)*mu_y1;

C_YA1s1 = theta_1s1*(1/3*beta_0p1*(1-e^3)-1/2*(1-e^2)*(mu_z1-lambda_01)-1/8*(1-e^2)*((3*alpha_1s1-beta_1c1)*mu_x1 + (alpha_1c1+3*beta_1s1)*mu_y1) - 1/2*(1-e)*beta_01*mu_x1*mu_y1)+...
        theta_1c1*(-1/8*(1-e^2)*((alpha_1c1-beta_1s1)*mu_x1 + (alpha_1s1+beta_1c1)*mu_y1) + 1/4*(1-e)*beta_01*(mu_x1^2-mu_y1^2)) + ...
        theta_01*(-1/3*(1-e^3)*alpha_1s1+1/2*(1-e^2)*(beta_0p1*mu_x1-beta_01*mu_y1) - mu_x1*(1-e)*(mu_z1-lambda_01) + 1/4*(1-e)*beta_1c1*(mu_x1^2-mu_y1^2)-1/2*(1-e)*beta_1s1*mu_x1*mu_y1)+...
        theta_t*(-alpha_1s1*(1/4-1/3*e)+1/3*(1-3/2*e+1/2*e^3)*(beta_0p1*mu_x1-beta_01*mu_y1)-1/2*mu_x1*(mu_z1-lambda_01)*(1+2*e-e^2)+1/8*beta_1c1*(mu_x1^2-mu_y1^2)*(1-e)^2+1/4*beta_1s1*mu_x1*mu_y1*(1-e)^2)+...
        alpha_1s1*(2/3*(1-e^3)*beta_0p1-(mu_z1-lambda_01)*(1-e^2)+3/4*beta_1s1*mu_y1*(1-e^2)+1/4*beta_1c1*mu_x1*(1-e^2))+...
        1/4*alpha_1c1*(1-e^2)*(beta_1s1*mu_x1-beta_1c1*mu_y1) + beta_01*mu_y1*(beta_0p1*(1-e^2)+2*(1-e)*(mu_z1-lambda_01)-3/2*beta_1s1*mu_y1*(1-e)+beta_1c1*mu_x1*(1-e))-...
        1/2*beta_01*beta_1s1*mu_x1^2*(1-e)+delta/a_0*(1-e^2)*mu_x1;

% 惯性力系数
Omega_dot = 0;
beta_0pp1 = 0; beta_1cpp1 = 0; beta_1spp1 = 0;
M_bar_beta = M_beta/(rho*pi*R^4);
epsilon_1c1 = beta_1cp1+beta_1s1-q_bar_H1; epsilon_1s1 = beta_1sp1+beta_1c1-p_bar_H1;
gamma_1c1 = beta_1cpp1 + Omega_dot/(Omega^2)*beta_1s1+2*beta_1sp1-beta_1c1-q_dot_H1/Omega^2-p_bar_H1;
gamma_1s1 = beta_1spp1 + Omega_dot/(Omega^2)*beta_1c1-2*beta_1cp1-beta_1s1-p_dot_H1/Omega^2+q_bar_H1;

C_XI01 = eta_z1*beta_01*m_bar_b + M_bar_beta*(1+beta_0p1^2+1/2*(epsilon_1s1^2+epsilon_1c1^2));
C_XI1c1= (eta_x1+eta_z1*beta_1c1)*m_bar_b + 1*beta_0p1*epsilon_1c1*M_bar_beta;
C_XI1s1= -(eta_y1-eta_z1*beta_1s1)*m_bar_b + 1*beta_0p1*epsilon_1s1*M_bar_beta;

C_YI01 = -M_bar_beta*(beta_01*beta_0p1-Omega_dot/Omega^2+1/2*epsilon_1c1*(beta_1c1-p_bar_H1)+1/2*epsilon_1s1*(beta_1s1+q_bar_H1));
C_YI1c1 = eta_y1*m_bar_b-M_bar_beta*(beta_01*epsilon_1c1+beta_0p1*(beta_1c1-p_bar_H1));
C_YI1s1 = eta_x1*m_bar_b-M_bar_beta*(beta_01*epsilon_1s1+beta_0p1*(beta_1s1+q_bar_H1));

C_ZI01 = -(eta_z1+1/2*(eta_y1*beta_1s1-eta_x1*beta_1c1))*m_bar_b+M_bar_beta*(beta_01+beta_0pp1);
C_ZI1c1 = eta_x1*beta_01*m_bar_b + M_bar_beta*(gamma_1c1+beta_1c1-p_bar_H1);
C_ZI1s1 = -eta_y1*beta_01*m_bar_b + M_bar_beta*(gamma_1s1+beta_1c1+q_bar_H1);

% 合力系数
C_X01 = C_XI01; C_X1c1 = C_XI1c1; C_X1s1 = C_XI1s1;
C_Y1c1 = s*a_0/(2*b)*C_YA1c1+C_YI1c1; C_Y1s1 = s*a_0/(2*b)*C_YA1s1+C_YI1s1;
C_Z01 = -s*a_0/(2*b)*C_ZA01+C_ZI01; C_Z1c1 = -s*a_0/(2*b)*C_ZA1c1+C_ZI1c1; C_Z1s1 = -s*a_0/(2*b)*C_ZA1s1+C_ZI1s1;

C_Xh1 = -b/2*(C_X1c1+C_Y1s1+beta_01*C_Z1c1+beta_1c1*C_Z01);
C_Yh1 = b/2*(C_X1s1+C_Y1c1+beta_01*C_Z1s1+beta_1s1*C_Z01);
C_Zh1 = b/2*(2*C_Z01+2*beta_01*C_Z01+beta_1c1*C_X1c1-beta_1s1*C_X1s1);

% 诱导速度升力系数
C_Zhi1 = 2*lambda_01*sqrt(mu_1^2+(mu_z1-lambda_01)^2);

Z_h1 = rho*(Omega*R)^2*A*C_Zh1;
Z_hi1 = rho*(Omega*R)^2*A*C_Zhi1;

% 合力
X_R1 = rho*(Omega*R)^2*A*(C_Xh1*cos(gamma_s) - C_Zh1*sin(gamma_s));
Y_R1 = rho*(Omega*R)^2*A*C_Yh1;
Z_R1 = rho*(Omega*R)^2*A*(C_Xh1*sin(gamma_s) + C_Zh1*cos(gamma_s));

% 气动力矩系数
C_QA01=theta_1c1*(-1/8*(1-e^4)*alpha_1c1+1/6*(1-e^3)*(beta_0p1*mu_y1+beta_01*mu_x1)+1/16*(1-e^2)*beta_1s1*(mu_x1^2-mu_y1^2)-1/4*mu_y1*(1-e^2)*...
        (mu_z1-lambda_01)+1/8*(1-e^2)*beta_1c1*mu_x1*mu_y1)+...
        theta_1s1*(-1/8*(1-e^4)*alpha_1s1+1/6*(1-e^3)*(beta_0p1*mu_x1-beta_01*mu_y1)+1/16*((1-e^2)*beta_1c1*(mu_x1^2-mu_y1^2))-1/4*mu_x1*(1-e^2)*...
        (mu_z1-lambda_01)-1/8*(1-e^2)*beta_1s1*mu_x1*mu_y1)+...
        theta_01*(1/4*(1-e^4)*beta_0p1-1/3*(1-e^3)*(mu_z1-lambda_01)-1/6*(1-e^3)*((alpha_1s1-beta_1c1)*mu_x1+(alpha_1c1+beta_1s1)*mu_y1))+...
        theta_t*((1/5-1/4*e+1/20*e^5)*beta_0p1-(1/4-1/3*e+1/12*e^4)*((mu_z1-lambda_01)-1/2*((alpha_1s1-beta_1c1)*mu_x1+(alpha_1c1+...
        beta_1s1)*mu_y1)))-...
        1/8*(1-e^4)*(alpha_1c1^2+alpha_1s1^2+2*beta_0p1^2)+(mu_z1-lambda_01)*(2/3*beta_0p1*(1-e^3)-1/2*(1-e^2)*((mu_z1-lambda_01)-(beta_1c1*mu_x1-...
        beta_1s1*mu_y1)))+1/3*(1-e^3)*(beta_01*(alpha_1c1*mu_x1-alpha_1s1*mu_y1)+beta_0p1*(beta_1s1*mu_y1-beta_1c1*mu_x1))+(1-e^2)*(-1/4*beta_01^2*mu_1^2+...
        1/4*beta_1s1*beta_1c1*mu_x1*mu_y1-1/16*mu_1^2*(beta_1s1^2+beta_1c1^2)-1/8*(beta_1c1^2*mu_x1^2+beta_1s1^2*mu_y1^2))+1/4*(1-e^2)*(1+e^2+mu_1^2)*delta/a_0;
    
C_QA1c1 = theta_1c1*(1/4*beta_0p1*(1-e^4)+(1-e^3)*(-1/3*(mu_z1-lambda_01)-1/12*(alpha_1s1*mu_x1+beta_1s1*mu_y1)+1/4*(beta_1c1*mu_x1-alpha_1c1*mu_y1))...
        +1/4*(1-e^2)*beta_01*mu_x1*mu_y1)+...
        theta_1s1*(1/8*(1-e^2)*beta_01*(mu_x1^2-mu_y1^2)+1/12*((1-e^3)*(mu_x1*(beta_1s1-alpha_1c1)-(alpha_1s1+beta_1c1)*mu_y1)))+...
        theta_01*(-1/4*(1-e^4)*alpha_1c1+1/3*(1-e^3)*(beta_0p1*mu_y1+beta_01*mu_x1)+1/2*(1-e^2)*(1/4*beta_1s1*(mu_x1^2-mu_y1^2)-mu_y1*(mu_z1-lambda_01)...
        +1/2*beta_1c1*mu_x1*mu_y1))+...
        theta_t*(-alpha_1c1*(1/5-1/4*e+1/20*e^5)+(1/4-1/3*e+1/12*e^4)*(beta_0p1*mu_y1+beta_01*mu_x1)+1/2*(1-e^3)*(1/2*beta_1s1*(mu_x1^2-...
        mu_y1^2)+beta_1c1*mu_x1*mu_y1-2*mu_y1*(mu_z1-lambda_01)))+...
        1/2*(1-e^4)*alpha_1c1*beta_0p1+(1-e^2)*(beta_01*mu_x1*(mu_z1-lambda_01)-1/2*beta_01*beta_1c1*(mu_x1^2-1/2*mu_1^2)+1/2*beta_1s1*beta_01*mu_x1*mu_y1)-...
        2/3*(1-e^3)*(alpha_1c1*(mu_z1-lambda_01)+beta_01*beta_0p1*mu_x1-delta*mu_y1/a_0+1/4*beta_1s1*(alpha_1c1*mu_y1-alpha_1s1*mu_x1)+1/4*beta_1c1*(alpha_1s1*mu_y1-...
        3*alpha_1c1*mu_x1));

C_QA1s1 = theta_1c1*(1/8*(1-e^2)*beta_01*(mu_x1^2-mu_y1^2)+1/12*(1-e^3)*(mu_x1*(beta_1s1-alpha_1c1)-(alpha_1s1+beta_1c1)*mu_y1))+...
        theta_1s1*(1/4*beta_0p1*(1-e^4)+(1-e^3)*(-1/3*(mu_z1-lambda_01)-1/4*(alpha_1s1*mu_x1+beta_1s1*mu_y1)+1/12*(beta_1c1*mu_x1-alpha_1c1*mu_y1))...
        -1/4*(1-e^2)*beta_01*mu_x1*mu_y1)+...
        theta_01*(-1/4*(1-e^4)*alpha_1s1+1/3*(1-e^3)*(beta_0p1*mu_x1-beta_01*mu_y1)+1/2*(1-e^2)*(1/4*beta_1c1*(mu_x1^2-mu_y1^2)-mu_x1*(mu_z1-lambda_01)...
        -1/2*beta_1s1*mu_x1*mu_y1))+...
        theta_t*(-alpha_1s1*(1/5-1/4*e+1/20*e^5)+(1/4-1/3*e+1/12*e^4)*(beta_0p1*mu_x1-beta_01*mu_y1)+1/2*(1-e^3)*(1/2*beta_1c1*(mu_x1^2-...
        mu_y1^2)-beta_1s1*mu_x1*mu_y1-2*mu_x1*(mu_z1-lambda_01)))+...
        1/2*(1-e^4)*alpha_1s1*beta_0p1+(1-e^2)*(-beta_01*mu_y1*(mu_z1-lambda_01)-1/2*beta_01*beta_1s1*(mu_y1^2+1/2*mu_1^2)+1/2*beta_1c1*beta_01*mu_x1*mu_y1)-...
        2/3*(1-e^3)*(alpha_1s1*(mu_z1-lambda_01)-beta_01*beta_0p1*mu_y1-delta*mu_x1/a_0+1/4*beta_1s1*(alpha_1c1*mu_x1-3*alpha_1s1*mu_y1)+1/4*beta_1c1*(alpha_1s1*mu_x1-...
        alpha_1c1*mu_y1));

% 惯性力矩系数
I_bar_beta = I_beta/(rho*pi*R^5);
C_QI01 = -I_bar_beta*(beta_01*beta_0p1-Omega_dot/Omega^2+1/2*epsilon_1c1*(beta_1c1-p_bar_H1)+1/2*epsilon_1s1*(beta_1s1+q_bar_H1));
C_QI1c1 = eta_y1*M_bar_beta - I_bar_beta*(beta_01*epsilon_1c1+beta_0p1*(beta_1c1-p_bar_H1));
C_QI1s1 = eta_x1*M_bar_beta - I_bar_beta*(beta_01*epsilon_1s1+beta_0p1*(beta_1s1+q_bar_H1));

% 合力矩系数
C_Q01 = s*a_0/(2*b)*C_QA01 + C_QI01;
C_Q1c1 = s*a_0/(2*b)*C_QA1c1 + C_QI1c1;
C_Q1s1 = s*a_0/(2*b)*C_QA1s1 + C_QI1s1;

% 合力矩
Q_01 = rho*(Omega*R)^2*A*R*C_Q01;
Q_1c1 = rho*(Omega*R)^2*A*R*C_Q1c1;
Q_1s1 = rho*(Omega*R)^2*A*R*C_Q1s1;

L_h1 = -b/2*(K_beta*beta_1s1+Q_01*beta_1c1+Q_1c1*beta_01);
M_h1 = b/2*(beta_1s1*Q_01+beta_01*Q_1s1-K_beta*beta_1c1);
N_h1 = b*Q_01;

L_R1 = L_h1*cos(gamma_s)-N_h1*sin(gamma_s)+h_R1*Y_R1;
M_R1 = M_h1-h_R1*X_R1+x_cg*Z_R1;
N_R1 = L_h1*sin(gamma_s)+N_h1*cos(gamma_s)-x_cg*Y_R1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 上旋翼模型
beta_0p2 = 0; beta_1sp2 = 0; beta_1cp2 = 0; 
mu_x2 = u_H2/(Omega*R); mu_y2 = v_H2/(Omega*R); mu_z2 = w_H2/(Omega*R); mu_2 = sqrt(mu_x2^2+mu_y2^2);
q_bar_H2 = q_H2/Omega; p_bar_H2 = p_H2/Omega;
lambda_02 = v_02/(Omega*R); lambda_1c2 = v_1c2/(Omega*R); lambda_1s2 = v_1s2/(Omega*R); 
a_Hx2 = (u_dot-(q_dot+(-p)*(-r))+(q^2+(-r)^2))*cos(gamma_s)+(w_dot+((-p)^2+q^2)+(q_dot-(-p)*(-r))*sin(gamma_s));
a_Hy2 = (-v_dot)+(p_dot-q*(-r))-(r_dot+(-p)*q);
a_Hz2 = (w_dot+((-p)^2+q^2)+(q_dot-(-p)*(-r)))*cos(gamma_s)-(u_dot-(q_dot+(-p)*(-r))+(q^2+(-r)^2))*sin(gamma_s);
m_bar_b = m_b/(rho*pi*R^3);
eta_x2 = a_Hx2/(Omega^2*R); eta_y2 = a_Hy2/(Omega^2*R); eta_z2 = a_Hz2/(Omega^2*R);

% 挥舞方程
n_beta = rho*c*a_0*R^4/(8*I_beta);
q_H1p2 = q_bar_H2/Omega^2; p_H1p2 = p_bar_H2/Omega^2;
lambda_beta = sqrt(1+K_beta/(I_beta*Omega^2));

hM02 = zeros(3,1);
hM02(1) = M_beta*R/I_beta*eta_z2+n_beta*(2/3*(1-e^3)*(2*(mu_z2-lambda_02)+((2*theta_1c2-lambda_1s2+p_bar_H2)*mu_y2+(2*theta_1s2-lambda_1c2+q_bar_H2)*mu_x2))+...
         (1-e^2)*(1+e^2+mu_2^2)*theta_01+(4/5-e+1/5*e^5+(2/3-e+1/3*e^3)*mu_2^2)*theta_t);

hM02(2) = 2*(p_bar_H2+q_H1p2/2)+n_beta*((1-e^2)*((1+e^2+1/2*mu_2^2+mu_y2^2)*theta_1c2+mu_x2*mu_y2*theta_1s2)+8/3*(1-e^3)*mu_y2*theta_01+...
        (2-8/3*e+2/3*e^4)*mu_y2*theta_t+2*(1-e^2)*(mu_z2-lambda_02)*mu_y2-(1-e^4)*(lambda_1c2-q_bar_H2));

hM02(3) = -2*(q_bar_H2-p_H1p2/2)+n_beta*((1-e^2)*((1+e^2+1/2*mu_2^2+mu_x2^2)*theta_1s2+mu_x2*mu_y2*theta_1c2)+8/3*(1-e^3)*mu_y2*theta_01+...
        (2-8/3*e+2/3*e^4)*mu_x2*theta_t+2*(1-e^2)*(mu_z2-lambda_02)*mu_x2-(1-e^4)*(lambda_1s2-p_bar_H2));

DM02 = zeros(3,3);
DM02(1,1) = lambda_beta^2;
DM02(1,2) = (M_beta*R/I_beta)*eta_x2/2;
DM02(1,3) = -(M_beta*R/I_beta)*eta_y2/2;
DM02(2,1) = (M_beta*R/I_beta)*eta_x2+4*mu_x2*n_beta/3 - 4*e^3*mu_x2*n_beta/3;
DM02(2,2) = mu_x2*mu_y2*n_beta*(1-e^2)+lambda_beta^2-1;
DM02(2,3) = n_beta*(1-e^2)*(2*e^2+mu_x2^2-mu_y2^2+2)/2;
DM02(3,1) = 4*mu_y2*n_beta*e^3/3 - (M_beta*R/I_beta)*eta_y2-4*mu_y2*n_beta/3;
DM02(3,2) = n_beta*(e^2-1)*(2*e^2-mu_x2^2+mu_y2^2+2)/2;
DM02(3,3) = mu_x2*mu_y2*n_beta*e^2+lambda_beta^2-mu_x2*mu_y2*n_beta-1;

flap2 = DM02\hM02;
beta_02 = flap2(1);
beta_1c2 = flap2(2);
beta_1s2 = flap2(3);

alpha_1c2 = q_bar_H2 - lambda_1c2 - beta_1cp2 - beta_1s2; alpha_1s2 = p_bar_H2 - lambda_1s2 - beta_1sp2 - beta_1c2; 
% 气动力系数
C_ZA02 = -1/3*beta_0p2*(1-e^3)+1/2*(1-e^2)*(mu_z2-lambda_02) + 1/4*(1-e^2)*(mu_x2*(alpha_1s2-beta_1c2)+mu_y2*(alpha_1c2+beta_1s2)) + ...
        1/2*(1-e^2)*(theta_1c2*mu_y2+theta_1s2*mu_x2) + (1/3*(1-e^3)+1/2*(1-e)*mu_2^2)*theta_01 + (1/4*(1+mu_2^2*(1-e)^2)-1/3*e)*theta_t;

C_ZA1c2 = 1/3*alpha_1c2*(1-e^3) + mu_y2*(1-e)*(mu_z2-lambda_02) - 1/2*(1-e^2)*(beta_0p2*mu_y2+beta_02*mu_x2) - 1/4*(1-e)*beta_1s2*(mu_x2^2-mu_y2^2) +...
        1/2*(1-e)*(theta_1s2-beta_1c2)*mu_x2*mu_y2 + (1/3*(1-e^3)+(1-e)*(1/4*mu_2^2+1/2*mu_y2^2))*theta_1c2 + mu_y2*(1-e^2)*theta_01 +...
        mu_y2*(2/3-e+1/3*e^3)*theta_t;

C_ZA1s2 = 1/3*alpha_1s2*(1-e^3) + mu_x2*(1-e)*(mu_z2-lambda_02) - 1/2*(1-e^2)*(beta_0p2*mu_x2+beta_02*mu_y2) - 1/4*(1-e)*beta_1c2*(mu_x2^2-mu_y2^2) +...
        1/2*(1-e)*(theta_1c2-beta_1s2)*mu_x2*mu_y2 + (1/3*(1-e^3)+(1-e)*(1/4*mu_2^2+1/2*mu_x2^2))*theta_1s2 + mu_x2*(1-e^2)*theta_01 +...
        mu_x2*(2/3-e+1/3*e^3)*theta_t;

C_YA1c2 = theta_1c2*(1/3*beta_0p2*(1-e^3)-1/2*(1-e^2)*(mu_z2-lambda_02)-1/8*(1-e^2)*((alpha_1s2-3*beta_1c2)*mu_x2 + (3*alpha_1c2+beta_1s2)*mu_y2) + 1/2*(1-e)*beta_02*mu_x2*mu_y2)+...
        theta_1s2*(-1/8*(1-e^2)*((alpha_1c2-beta_1s2)*mu_x2 + (alpha_1s2+beta_1c2)*mu_y2) + 1/4*(1-e)*beta_02*(mu_x2^2-mu_y2^2)) + ...
        theta_01*(-1/3*(1-e^3)*alpha_1c2+1/2*(1-e^2)*(beta_0p2*mu_y2+beta_02*mu_x2) - mu_y2*(1-e)*(mu_z2-lambda_02) + 1/4*(1-e)*beta_1s2*(mu_x2^2-mu_y2^2)+1/2*(1-e)*beta_1c2*mu_x2*mu_y2)+...
        theta_t*(-alpha_1c2*(1/4-1/3*e)+1/3*(1-3/2*e+1/2*e^3)*(beta_0p2*mu_y2+beta_02*mu_x2)-1/2*mu_y2*(mu_z2-lambda_02)*(1-e)^2+1/8*beta_1s2*(mu_x2^2-mu_y2^2)*(1-e)^2+1/4*beta_1c2*mu_x2*mu_y2*(1-e)^2)+...
        alpha_1c2*(2/3*(1-e^3)*beta_0p2-(mu_z2-lambda_02)*(1-e^2)+3/4*beta_1c2*mu_x2*(1-e^2)-1/4*beta_1s2*mu_y2*(1-e^2))+...
        1/4*alpha_1s2*(1-e^2)*(beta_1s2*mu_x2-beta_1c2*mu_y2) + beta_02*mu_x2*(-beta_0p2*(1-e^2)+2*(1-e)*(mu_z2-lambda_02)-3/2*beta_1c2*mu_x2*(1-e)+beta_1s2*mu_y2*(1-e))-...
        1/2*beta_02*beta_1c2*mu_y2^2*(1-e)+delta/a_0*(1-e^2)*mu_y2;

C_YA1s2 = theta_1s2*(1/3*beta_0p2*(1-e^3)-1/2*(1-e^2)*(mu_z2-lambda_02)-1/8*(1-e^2)*((3*alpha_1s2-beta_1c2)*mu_x2 + (alpha_1c2+3*beta_1s2)*mu_y2) - 1/2*(1-e)*beta_02*mu_x2*mu_y2)+...
        theta_1c2*(-1/8*(1-e^2)*((alpha_1c2-beta_1s2)*mu_x2 + (alpha_1s2+beta_1c2)*mu_y2) + 1/4*(1-e)*beta_02*(mu_x2^2-mu_y2^2)) + ...
        theta_01*(-1/3*(1-e^3)*alpha_1s2+1/2*(1-e^2)*(beta_0p2*mu_x2-beta_02*mu_y2) - mu_x2*(1-e)*(mu_z2-lambda_02) + 1/4*(1-e)*beta_1c2*(mu_x2^2-mu_y2^2)-1/2*(1-e)*beta_1s2*mu_x2*mu_y2)+...
        theta_t*(-alpha_1s2*(1/4-1/3*e)+1/3*(1-3/2*e+1/2*e^3)*(beta_0p2*mu_x2-beta_02*mu_y2)-1/2*mu_x2*(mu_z2-lambda_02)*(1+2*e-e^2)+1/8*beta_1c2*(mu_x2^2-mu_y2^2)*(1-e)^2+1/4*beta_1s2*mu_x2*mu_y2*(1-e)^2)+...
        alpha_1s2*(2/3*(1-e^3)*beta_0p2-(mu_z2-lambda_02)*(1-e^2)+3/4*beta_1s2*mu_y2*(1-e^2)+1/4*beta_1c2*mu_x2*(1-e^2))+...
        1/4*alpha_1c2*(1-e^2)*(beta_1s2*mu_x2-beta_1c2*mu_y2) + beta_02*mu_y2*(beta_0p2*(1-e^2)+2*(1-e)*(mu_z2-lambda_02)-3/2*beta_1s2*mu_y2*(1-e)+beta_1c2*mu_x2*(1-e))-...
        1/2*beta_02*beta_1s2*mu_x2^2*(1-e)+delta/a_0*(1-e^2)*mu_x2;

% 惯性力系数
Omega_dot = 0;
beta_0pp2 = 0; beta_1cpp2 = 0; beta_1spp2 = 0;
M_bar_beta = M_beta/(rho*pi*R^4);
epsilon_1c2 = beta_1cp2+beta_1s2-q_bar_H2; epsilon_1s2 = beta_1sp2+beta_1c2-p_bar_H2;
gamma_1c2 = beta_1cpp2 + Omega_dot/(Omega^2)*beta_1s2+2*beta_1sp2-beta_1c2-q_dot_H2/Omega^2-p_bar_H2;
gamma_1s2 = beta_1spp2 + Omega_dot/(Omega^2)*beta_1c2-2*beta_1cp2-beta_1s2-p_dot_H2/Omega^2+q_bar_H2;

C_XI02 = eta_z2*beta_02*m_bar_b + M_bar_beta*(1+beta_0p2^2+1/2*(epsilon_1s2^2+epsilon_1c2^2));
C_XI1c2= (eta_x2+eta_z2*beta_1c2)*m_bar_b + 1*beta_0p2*epsilon_1c2*M_bar_beta;
C_XI1s2= -(eta_y2-eta_z2*beta_1s2)*m_bar_b + 1*beta_0p2*epsilon_1s2*M_bar_beta;

C_YI02 = -M_bar_beta*(beta_02*beta_0p2-Omega_dot/Omega^2+1/2*epsilon_1c2*(beta_1c2-p_bar_H2)+1/2*epsilon_1s2*(beta_1s2+q_bar_H2));
C_YI1c2 = eta_y2*m_bar_b-M_bar_beta*(beta_02*epsilon_1c2+beta_0p2*(beta_1c2-p_bar_H2));
C_YI1s2 = eta_x2*m_bar_b-M_bar_beta*(beta_02*epsilon_1s2+beta_0p2*(beta_1s2+q_bar_H2));

C_ZI02 = -(eta_z2+1/2*(eta_y2*beta_1s2-eta_x2*beta_1c2))*m_bar_b+M_bar_beta*(beta_02+beta_0pp2);
C_ZI1c2 = eta_x2*beta_02*m_bar_b + M_bar_beta*(gamma_1c2+beta_1c2-p_bar_H2);
C_ZI1s2 = -eta_y2*beta_02*m_bar_b + M_bar_beta*(gamma_1s2+beta_1c2+q_bar_H2);

% 合力系数
C_X02 = C_XI02; C_X1c2 = C_XI1c2; C_X1s2 = C_XI1s2;
C_Y1c2 = s*a_0/(2*b)*C_YA1c2+C_YI1c2; C_Y1s2 = s*a_0/(2*b)*C_YA1s2+C_YI1s2;
C_Z02 = -s*a_0/(2*b)*C_ZA02+C_ZI02; C_Z1c2 = -s*a_0/(2*b)*C_ZA1c2+C_ZI1c2; C_Z1s2 = -s*a_0/(2*b)*C_ZA1s2+C_ZI1s2;

C_Xh2 = -b/2*(C_X1c2+C_Y1s2+beta_02*C_Z1c2+beta_1c2*C_Z02);
C_Yh2 = b/2*(C_X1s2+C_Y1c2+beta_02*C_Z1s2+beta_1s2*C_Z02);
C_Zh2 = b/2*(2*C_Z02+2*beta_02*C_Z02+beta_1c2*C_X1c2-beta_1s2*C_X1s2);

% 诱导速度升力系数
C_Zhi2 = 2*lambda_02*sqrt(mu_2^2+(mu_z2-lambda_02)^2);

Z_h2 = rho*(Omega*R)^2*A*C_Zh2;
Z_hi2 = rho*(Omega*R)^2*A*C_Zhi2;

% 合力
X_R2 = rho*(Omega*R)^2*A*(C_Xh2*cos(gamma_s) - C_Zh2*sin(gamma_s));
Y_R2 = rho*(Omega*R)^2*A*C_Yh2;
Z_R2 = rho*(Omega*R)^2*A*(C_Xh2*sin(gamma_s) + C_Zh2*cos(gamma_s));

% 气动力矩系数
C_QA02=theta_1c2*(-1/8*(1-e^4)*alpha_1c2+1/6*(1-e^3)*(beta_0p2*mu_y2+beta_02*mu_x2)+1/16*(1-e^2)*beta_1s2*(mu_x2^2-mu_y2^2)-1/4*mu_y2*(1-e^2)*...
        (mu_z2-lambda_02)+1/8*(1-e^2)*beta_1c2*mu_x2*mu_y2)+...
        theta_1s2*(-1/8*(1-e^4)*alpha_1s2+1/6*(1-e^3)*(beta_0p2*mu_x2-beta_02*mu_y2)+1/16*((1-e^2)*beta_1c2*(mu_x2^2-mu_y2^2))-1/4*mu_x2*(1-e^2)*...
        (mu_z2-lambda_02)-1/8*(1-e^2)*beta_1s2*mu_x2*mu_y2)+...
        theta_01*(1/4*(1-e^4)*beta_0p2-1/3*(1-e^3)*(mu_z2-lambda_02)-1/6*(1-e^3)*((alpha_1s2-beta_1c2)*mu_x2+(alpha_1c2+beta_1s2)*mu_y2))+...
        theta_t*((1/5-1/4*e+1/20*e^5)*beta_0p2-(1/4-1/3*e+1/12*e^4)*((mu_z2-lambda_02)-1/2*((alpha_1s2-beta_1c2)*mu_x2+(alpha_1c2+...
        beta_1s2)*mu_y2)))-...
        1/8*(1-e^4)*(alpha_1c2^2+alpha_1s2^2+2*beta_0p2^2)+(mu_z2-lambda_02)*(2/3*beta_0p2*(1-e^3)-1/2*(1-e^2)*((mu_z2-lambda_02)-(beta_1c2*mu_x2-...
        beta_1s2*mu_y2)))+1/3*(1-e^3)*(beta_02*(alpha_1c2*mu_x2-alpha_1s2*mu_y2)+beta_0p2*(beta_1s2*mu_y2-beta_1c2*mu_x2))+(1-e^2)*(-1/4*beta_02^2*mu_2^2+...
        1/4*beta_1s2*beta_1c2*mu_x2*mu_y2-1/16*mu_2^2*(beta_1s2^2+beta_1c2^2)-1/8*(beta_1c2^2*mu_x2^2+beta_1s2^2*mu_y2^2))+1/4*(1-e^2)*(1+e^2+mu_2^2)*delta/a_0;
    
C_QA1c2 = theta_1c2*(1/4*beta_0p2*(1-e^4)+(1-e^3)*(-1/3*(mu_z2-lambda_02)-1/12*(alpha_1s2*mu_x2+beta_1s2*mu_y2)+1/4*(beta_1c2*mu_x2-alpha_1c2*mu_y2))...
        +1/4*(1-e^2)*beta_02*mu_x2*mu_y2)+...
        theta_1s2*(1/8*(1-e^2)*beta_02*(mu_x2^2-mu_y2^2)+1/12*((1-e^3)*(mu_x2*(beta_1s2-alpha_1c2)-(alpha_1s2+beta_1c2)*mu_y2)))+...
        theta_01*(-1/4*(1-e^4)*alpha_1c2+1/3*(1-e^3)*(beta_0p2*mu_y2+beta_02*mu_x2)+1/2*(1-e^2)*(1/4*beta_1s2*(mu_x2^2-mu_y2^2)-mu_y2*(mu_z2-lambda_02)...
        +1/2*beta_1c2*mu_x2*mu_y2))+...
        theta_t*(-alpha_1c2*(1/5-1/4*e+1/20*e^5)+(1/4-1/3*e+1/12*e^4)*(beta_0p2*mu_y2+beta_02*mu_x2)+1/2*(1-e^3)*(1/2*beta_1s2*(mu_x2^2-...
        mu_y2^2)+beta_1c2*mu_x2*mu_y2-2*mu_y2*(mu_z2-lambda_02)))+...
        1/2*(1-e^4)*alpha_1c2*beta_0p2+(1-e^2)*(beta_02*mu_x2*(mu_z2-lambda_02)-1/2*beta_02*beta_1c2*(mu_x2^2-1/2*mu_2^2)+1/2*beta_1s2*beta_02*mu_x2*mu_y2)-...
        2/3*(1-e^3)*(alpha_1c2*(mu_z2-lambda_02)+beta_02*beta_0p2*mu_x2-delta*mu_y2/a_0+1/4*beta_1s2*(alpha_1c2*mu_y2-alpha_1s2*mu_x2)+1/4*beta_1c2*(alpha_1s2*mu_y2-...
        3*alpha_1c2*mu_x2));

C_QA1s2 = theta_1c2*(1/8*(1-e^2)*beta_02*(mu_x2^2-mu_y2^2)+1/12*(1-e^3)*(mu_x2*(beta_1s2-alpha_1c2)-(alpha_1s2+beta_1c2)*mu_y2))+...
        theta_1s2*(1/4*beta_0p2*(1-e^4)+(1-e^3)*(-1/3*(mu_z2-lambda_02)-1/4*(alpha_1s2*mu_x2+beta_1s2*mu_y2)+1/12*(beta_1c2*mu_x2-alpha_1c2*mu_y2))...
        -1/4*(1-e^2)*beta_02*mu_x2*mu_y2)+...
        theta_01*(-1/4*(1-e^4)*alpha_1s2+1/3*(1-e^3)*(beta_0p2*mu_x2-beta_02*mu_y2)+1/2*(1-e^2)*(1/4*beta_1c2*(mu_x2^2-mu_y2^2)-mu_x2*(mu_z2-lambda_02)...
        -1/2*beta_1s2*mu_x2*mu_y2))+...
        theta_t*(-alpha_1s2*(1/5-1/4*e+1/20*e^5)+(1/4-1/3*e+1/12*e^4)*(beta_0p2*mu_x2-beta_02*mu_y2)+1/2*(1-e^3)*(1/2*beta_1c2*(mu_x2^2-...
        mu_y2^2)-beta_1s2*mu_x2*mu_y2-2*mu_x2*(mu_z2-lambda_02)))+...
        1/2*(1-e^4)*alpha_1s2*beta_0p2+(1-e^2)*(-beta_02*mu_y2*(mu_z2-lambda_02)-1/2*beta_02*beta_1s2*(mu_y2^2+1/2*mu_2^2)+1/2*beta_1c2*beta_02*mu_x2*mu_y2)-...
        2/3*(1-e^3)*(alpha_1s2*(mu_z2-lambda_02)-beta_02*beta_0p2*mu_y2-delta*mu_x2/a_0+1/4*beta_1s2*(alpha_1c2*mu_x2-3*alpha_1s2*mu_y2)+1/4*beta_1c2*(alpha_1s2*mu_x2-...
        alpha_1c2*mu_y2));

% 惯性力矩系数
I_bar_beta = I_beta/(rho*pi*R^5);
C_QI02 = -I_bar_beta*(beta_02*beta_0p2-Omega_dot/Omega^2+1/2*epsilon_1c2*(beta_1c2-p_bar_H2)+1/2*epsilon_1s2*(beta_1s2+q_bar_H2));
C_QI1c2 = eta_y2*M_bar_beta - I_bar_beta*(beta_02*epsilon_1c2+beta_0p2*(beta_1c2-p_bar_H2));
C_QI1s2 = eta_x2*M_bar_beta - I_bar_beta*(beta_02*epsilon_1s2+beta_0p2*(beta_1s2+q_bar_H2));

% 合力矩系数
C_Q02 = s*a_0/(2*b)*C_QA02 + C_QI02;
C_Q1c2 = s*a_0/(2*b)*C_QA1c2 + C_QI1c2;
C_Q1s2 = s*a_0/(2*b)*C_QA1s2 + C_QI1s2;

% 合力矩
Q_02 = rho*(Omega*R)^2*A*R*C_Q02;
Q_1c2 = rho*(Omega*R)^2*A*R*C_Q1c2;
Q_1s2 = rho*(Omega*R)^2*A*R*C_Q1s2;

L_h2 = -b/2*(K_beta*beta_1s2+Q_02*beta_1c2+Q_1c2*beta_02);
M_h2 = b/2*(beta_1s2*Q_02+beta_02*Q_1s2-K_beta*beta_1c2);
N_h2 = b*Q_02;

L_R2 = L_h2*cos(gamma_s)-N_h2*sin(gamma_s)+h_R2*Y_R2;
M_R2 = M_h2-h_R2*X_R2+x_cg*Z_R2;
N_R2 = L_h2*sin(gamma_s)+N_h2*cos(gamma_s)-x_cg*Y_R2;

L_R2 = -L_R2;
M_R2 = M_R2;
N_R2 = -N_R2;

%% 合力
X_R = X_R1+X_R2;
Y_R = Y_R1+Y_R2;
Z_R = Z_R1+Z_R2;
L_R = L_R1+L_R2;
M_R = M_R1+M_R2;
N_R = N_R1+N_R2;

F(1) = Z_h1 + Z_hi1;
F(2) = Z_h2 + Z_hi2;
F(3) = X_R;
F(4) = Y_R;
F(5) = Z_R+55000;



Rotor.X_R1 = X_R1;
Rotor.Y_R1 = Y_R1;
Rotor.Z_R1 = Z_R1;
Rotor.L_R1 = L_R1;
Rotor.M_R1 = M_R1;
Rotor.N_R1 = N_R1;
Rotor.v_01 = v_01;
Rotor.beta_01 = beta_01;
Rotor.beta_1c1 = beta_1c1;
Rotor.beta_1s1 = beta_1s1;
Rotor.theta_01 = theta_01;
Rotor.theta_1c1 = theta_1c1;
Rotor.theta_1s1 = theta_1s1;
Rotor.X_R2 = X_R2;
Rotor.Y_R2 = Y_R2;
Rotor.Z_R2 = Z_R2;
Rotor.L_R2 = L_R2;
Rotor.M_R2 = M_R2;
Rotor.N_R2 = N_R2;
Rotor.v_02 = v_02;
Rotor.beta_02 = beta_02;
Rotor.beta_1c2 = beta_1c2;
Rotor.beta_1s2 = beta_1s2;
Rotor.theta_02 = theta_02;
Rotor.theta_1c2 = theta_1c2;
Rotor.theta_1s2 = theta_1s2;
Rotor.X_R = X_R;
Rotor.Y_R = Y_R;
Rotor.Z_R = Z_R;
Rotor.L_R = L_R;
Rotor.M_R = M_R;
Rotor.N_R = N_R;
%
end