function LowerRotor = CalculateLowerRotor(RotorStates)
% 计算下旋翼气动力，顶视逆时针转
% 输入为结构体: RotorStates, 下旋翼状态; 
% 字段: 
%   RotorStates.theta_01
%   RotorStates.theta_1c1
%   RotorStates.theta_1s1
%   RotorStates.v_01

global rho
global Rotor b m_b R A Omega s theta_t I_beta a_0 delta c K_beta e M_beta gamma_s h_R x_cg

global HeliStates

%% 需要输入的变量
% 运动状态
u_H1=RotorStates.u_H1; 
v_H1=RotorStates.v_H1; 
w_H1=RotorStates.w_H1;
p_H1=RotorStates.p_H1; 
q_H1=RotorStates.q_H1;

u_dot=HeliStates.u_dot; 
v_dot=HeliStates.v_dot; 
w_dot=HeliStates.w_dot;
p=HeliStates.p; 
q=HeliStates.q; 
r=HeliStates.r;
p_dot=HeliStates.p_dot; 
q_dot=HeliStates.q_dot; 
r_dot=HeliStates.r_dot;

% 操纵变量
theta_01 = RotorStates.theta_01;
theta_1c1= RotorStates.theta_1c1; 
theta_1s1= RotorStates.theta_1s1;
% 诱导速度
v_01 = RotorStates.v_01; 
v_1c1 = 0; v_1s1 = 0;

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
beta_0p1p = 0; beta_1cp1p = 0; beta_1sp1p = 0;
M_bar_beta = M_beta/(rho*pi*R^4);
epsilon_1c1 = beta_1cp1+beta_1s1-q_bar_H1; epsilon_1s1 = beta_1sp1+beta_1c1-p_bar_H1;
gamma_1c1 = beta_1cp1p + Omega_dot/(Omega^2)*beta_1s1+2*beta_1sp1-beta_1c1-q_dot_H/Omega^2-p_bar_H1;
gamma_1s1 = beta_1sp1p + Omega_dot/(Omega^2)*beta_1c1-2*beta_1cp1-beta_1s1-p_dot_H/Omega^2+q_bar_H1;

C_XI01 = eta_z1*beta_01*m_bar_b + M_bar_beta*(1+beta_0p1^2+1/2*(epsilon_1s1^2+epsilon_1c1^2));
C_XI1c1= (eta_x1+eta_z1*beta_1c1)*m_bar_b + 1*beta_0p1*epsilon_1c1*M_bar_beta;
C_XI1s1= -(eta_y1-eta_z1*beta_1s1)*m_bar_b + 1*beta_0p1*epsilon_1s1*M_bar_beta;

C_YI01 = -M_bar_beta*(beta_01*beta_0p1-Omega_dot/Omega^2+1/2*epsilon_1c1*(beta_1c1-p_bar_H1)+1/2*epsilon_1s1*(beta_1s1+q_bar_H1));
C_YI1c1 = eta_y1*m_bar_b-M_bar_beta*(beta_01*epsilon_1c1+beta_0p1*(beta_1c1-p_bar_H1));
C_YI1s1 = eta_x1*m_bar_b-M_bar_beta*(beta_01*epsilon_1s1+beta_0p1*(beta_1s1+q_bar_H1));

C_ZI01 = -(eta_z1+1/2*(eta_y1*beta_1s1-eta_x1*beta_1c1))*m_bar_b+M_bar_beta*(beta_01+beta_0p1p);
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

L_R1 = L_h1*cos(gamma_s)-N_h1*sin(gamma_s)+h_R*Y_R1;
M_R1 = M_h1-h_R*X_R1+x_cg*Z_R1;
N_R1 = L_h1*sin(gamma_s)+N_h1*cos(gamma_s)-x_cg*Y_R1;

LowerRotor.v_01 = v_01
LowerRotor.Z_h1 = Z_h1
LowerRotor.Z_hi1 = Z_hi1

end
