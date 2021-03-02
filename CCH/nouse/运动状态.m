% 运动状态
u_H2=u_H1; v_H2=-v_H1; w_H2=w_H2;
u_dot=0; v_dot=0; w_dot=0;
p=0; q=0; r=0;
p_H2=-p_H1; q_H2=q_H1;
p_dot=0; q_dot=0; r_dot=0;
p_dot_H2=-p_dot_H1; q_dot_H2=q_dot_H1; 
%操纵变量 1c:纵向力 1s：横向力
theta_1c2=0; theta_1s2=0;
%诱导速度
v_02 = States; v_1c2 = 0; v_1s2 = 0;

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

L_R2 = L_h2*cos(gamma_s)-N_h2*sin(gamma_s)+h_R*Y_R2;
M_R2 = M_h2-h_R*X_R2+x_cg*Z_R2;
N_R2 = L_h2*sin(gamma_s)+N_h2*cos(gamma_s)-x_cg*Y_R2;