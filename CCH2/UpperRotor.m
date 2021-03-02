function F = UpperRotor(State)
% 旋翼特性基本测试：模拟旋转塔，无线速度，角速度，横纵变距为0。给定总距，计算受力。
global rho
global Rotor b m_b R A Omega s theta_t I_beta a_0 delta c K_beta e M_beta gamma_s h_R2 x_cg

global theta_0

v_0 = State;

theta_1c=0; theta_1s=0.01;

u_H=0; v_H=0; w_H=0;
u_dot=0; v_dot=0; w_dot=0;
p=0; q=0; r=0;
p_H=0; q_H=0;
p_dot=0; q_dot=0; r_dot=0;
q_dot_H=0; p_dot_H=0;



%% Main Rotor Suppose only lower rotor
beta_0p = 0; beta_1sp = 0; beta_1cp = 0; 
% v_1c = 0; v_1s = 0;
mu_x = u_H/(Omega*R); mu_y = v_H/(Omega*R); mu_z = w_H/(Omega*R); mu = sqrt(mu_x^2+mu_y^2);
q_bar_H = q_H/Omega; p_bar_H = p_H/Omega;
lambda_0 = v_0/(Omega*R); 
% lambda_1c = v_1c/(Omega*R); lambda_1s = v_1s/(Omega*R); 
a_Hx = (u_dot-(q_dot+p*r)+(q^2+r^2))*cos(gamma_s)+(w_dot+(p^2+q^2)+(q_dot-p*r)*sin(gamma_s));
a_Hy = v_dot+(p_dot-q*r)-(r_dot+p*q);
a_Hz = (w_dot+(p^2+q^2)+(q_dot-p*r))*cos(gamma_s)-(u_dot-(q_dot+p*r)+(q^2+r^2))*sin(gamma_s);
m_bar_b = m_b/(rho*pi*R^3);
eta_x = a_Hx/(Omega^2*R); eta_y = a_Hy/(Omega^2*R); eta_z = a_Hz/(Omega^2*R);

if mu_x == 0
    phi_W = atan(mu_y/1e-10);
else
    phi_W = atan(mu_y/mu_x);
end

Chi_W = atan(mu/(lambda_0-mu_z));   

if Chi_W < pi/2
    lambda_1cW = lambda_0*tan(Chi_W/2);
else
    lambda_1cW = lambda_0*cot(Chi_W/2);
end

    
lambda_1c =  lambda_1cW*cos(phi_W);
lambda_1s =  lambda_1cW*sin(phi_W);

v_1c = lambda_1c*Omega*R;
v_1s = lambda_1s*Omega*R;

% 挥舞方程
n_beta = rho*c*a_0*R^4/(8*I_beta);
q_Hp = q_bar_H/Omega^2; p_Hp = p_bar_H/Omega^2;
lambda_beta = sqrt(1+K_beta/(I_beta*Omega^2));

hM0 = zeros(3,1);


hM0(1) = M_beta*R/I_beta*eta_z+n_beta*(2/3*(1-e^3)*(2*(mu_z-lambda_0)+((2*theta_1c-lambda_1s+p_bar_H)*mu_y+(2*theta_1s-lambda_1c+q_bar_H)*mu_x))+...
         (1-e^2)*(1+e^2+mu^2)*theta_0+(4/5-e+1/5*e^5+(2/3-e+1/3*e^3)*mu^2)*theta_t);

hM0(2) = 2*(p_bar_H+q_Hp/2)+n_beta*((1-e^2)*((1+e^2+1/2*mu^2+mu_y^2)*theta_1c+mu_x*mu_y*theta_1s)+8/3*(1-e^3)*mu_y*theta_0+...
        (2-8/3*e+2/3*e^4)*mu_y*theta_t+2*(1-e^2)*(mu_z-lambda_0)*mu_y-(1-e^4)*(lambda_1c-q_bar_H));

hM0(3) = -2*(q_bar_H-p_Hp/2)+n_beta*((1-e^2)*((1+e^2+1/2*mu^2+mu_x^2)*theta_1s+mu_x*mu_y*theta_1c)+8/3*(1-e^3)*mu_y*theta_0+...
        (2-8/3*e+2/3*e^4)*mu_x*theta_t+2*(1-e^2)*(mu_z-lambda_0)*mu_x-(1-e^4)*(lambda_1s-p_bar_H));

DM0 = zeros(3,3);
DM0(1,1) = lambda_beta^2;
DM0(1,2) = (M_beta*R/I_beta)*eta_x/2;
DM0(1,3) = -(M_beta*R/I_beta)*eta_y/2;
DM0(2,1) = (M_beta*R/I_beta)*eta_x+4*mu_x*n_beta/3 - 4*e^3*mu_x*n_beta/3;
DM0(2,2) = mu_x*mu_y*n_beta*(1-e^2)+lambda_beta^2-1;
DM0(2,3) = n_beta*(1-e^2)*(2*e^2+mu_x^2-mu_y^2+2)/2;
DM0(3,1) = 4*mu_y*n_beta*e^3/3 - (M_beta*R/I_beta)*eta_y-4*mu_y*n_beta/3;
DM0(3,2) = n_beta*(e^2-1)*(2*e^2-mu_x^2+mu_y^2+2)/2;
DM0(3,3) = mu_x*mu_y*n_beta*e^2+lambda_beta^2-mu_x*mu_y*n_beta-1;

% hM0 = zeros(4,1);
% 
% 
% hM0(1) = M_beta*R/I_beta*eta_z+n_beta*(2/3*(1-e^3)*(2*(mu_z-lambda_0)+((2*theta_1c-lambda_1s+p_bar_H)*mu_y+(2*theta_1s-lambda_1c+q_bar_H)*mu_x))+...
%          (1-e^2)*(1+e^2+mu^2)*theta_0+(4/5-e+1/5*e^5+(2/3-e+1/3*e^3)*mu^2)*theta_t);
% 
% hM0(3) = 2*(p_bar_H+q_Hp/2)+n_beta*((1-e^2)*((1+e^2+1/2*mu^2+mu_y^2)*theta_1c+mu_x*mu_y*theta_1s)+8/3*(1-e^3)*mu_y*theta_0+...
%         (2-8/3*e+2/3*e^4)*mu_y*theta_t+2*(1-e^2)*(mu_z-lambda_0)*mu_y-(1-e^4)*(lambda_1c-q_bar_H));
% 
% hM0(4) = -2*(q_bar_H-p_Hp/2)+n_beta*((1-e^2)*((1+e^2+1/2*mu^2+mu_x^2)*theta_1s+mu_x*mu_y*theta_1c)+8/3*(1-e^3)*mu_y*theta_0+...
%         (2-8/3*e+2/3*e^4)*mu_x*theta_t+2*(1-e^2)*(mu_z-lambda_0)*mu_x-(1-e^4)*(lambda_1s-p_bar_H));
% 
% DM0 = zeros(4,4);
% DM0(1,1) = lambda_beta^2;
% DM0(1,3) = (M_beta*R/I_beta)*eta_x/2;
% DM0(1,4) = -(M_beta*R/I_beta)*eta_y/2;
% DM0(2,2) = lambda_beta^2;
% DM0(3,1) = (M_beta*R/I_beta)*eta_x+4/3*mu_x*(1-e^3)*n_beta;
% DM0(3,3) = lambda_beta^2-1+mu_x*mu_y*n_beta*(1-e^2);
% DM0(3,4) = (1-e^4+1/2*(1-e^2)*(mu_x^2-mu_y^2))*n_beta;
% DM0(4,1) = -(M_beta*R/I_beta)*eta_y+4/3*mu_y*(1-e^3)*n_beta;
% DM0(4,3) = -(1-e^4+1/2*(1-e^2)*(mu_x^2-mu_y^2))*n_beta;
% DM0(4,4) = lambda_beta^2-1-mu_x*mu_y*n_beta*(1-e^2);

flap = inv(DM0)*hM0;
beta_0 = flap(1);
beta_1c = flap(2);
beta_1s = flap(3);

% beta_1c = -0.01;
% beta_1s = -0.01;

alpha_1c = q_bar_H - lambda_1c - beta_1cp - beta_1s; alpha_1s = p_bar_H - lambda_1s - beta_1sp - beta_1c; 
% 气动力系数
C_ZA0 = -1/3*beta_0p*(1-e^3)+1/2*(1-e^2)*(mu_z-lambda_0) + 1/4*(1-e^2)*(mu_x*(alpha_1s-beta_1c)+mu_y*(alpha_1c+beta_1s)) + ...
        1/2*(1-e^2)*(theta_1c*mu_y+theta_1s*mu_x) + (1/3*(1-e^3)+1/2*(1-e)*mu^2)*theta_0 + (1/4*(1+mu^2*(1-e)^2)-1/3*e)*theta_t;

C_ZA1c = 1/3*alpha_1c*(1-e^3) + mu_y*(1-e)*(mu_z-lambda_0) - 1/2*(1-e^2)*(beta_0p*mu_y+beta_0*mu_x) - 1/4*(1-e)*beta_1s*(mu_x^2-mu_y^2) +...
        1/2*(1-e)*(theta_1s-beta_1c)*mu_x*mu_y + (1/3*(1-e^3)+(1-e)*(1/4*mu^2+1/2*mu_y^2))*theta_1c + mu_y*(1-e^2)*theta_0 +...
        mu_y*(2/3-e+1/3*e^3)*theta_t;

C_ZA1s = 1/3*alpha_1s*(1-e^3) + mu_x*(1-e)*(mu_z-lambda_0) - 1/2*(1-e^2)*(beta_0p*mu_x+beta_0*mu_y) - 1/4*(1-e)*beta_1c*(mu_x^2-mu_y^2) +...
        1/2*(1-e)*(theta_1c-beta_1s)*mu_x*mu_y + (1/3*(1-e^3)+(1-e)*(1/4*mu^2+1/2*mu_x^2))*theta_1s + mu_x*(1-e^2)*theta_0 +...
        mu_x*(2/3-e+1/3*e^3)*theta_t;

C_YA1c = theta_1c*(1/3*beta_0p*(1-e^3)-1/2*(1-e^2)*(mu_z-lambda_0)-1/8*(1-e^2)*((alpha_1s-3*beta_1c)*mu_x + (3*alpha_1c+beta_1s)*mu_y) + 1/2*(1-e)*beta_0*mu_x*mu_y)+...
        theta_1s*(-1/8*(1-e^2)*((alpha_1c-beta_1s)*mu_x + (alpha_1s+beta_1c)*mu_y) + 1/4*(1-e)*beta_0*(mu_x^2-mu_y^2)) + ...
        theta_0*(-1/3*(1-e^3)*alpha_1c+1/2*(1-e^2)*(beta_0p*mu_y+beta_0*mu_x) - mu_y*(1-e)*(mu_z-lambda_0) + 1/4*(1-e)*beta_1s*(mu_x^2-mu_y^2)+1/2*(1-e)*beta_1c*mu_x*mu_y)+...
        theta_t*(-alpha_1c*(1/4-1/3*e)+1/3*(1-3/2*e+1/2*e^3)*(beta_0p*mu_y+beta_0*mu_x)-1/2*mu_y*(mu_z-lambda_0)*(1-e)^2+1/8*beta_1s*(mu_x^2-mu_y^2)*(1-e)^2+1/4*beta_1c*mu_x*mu_y*(1-e)^2)+...
        alpha_1c*(2/3*(1-e^3)*beta_0p-(mu_z-lambda_0)*(1-e^2)+3/4*beta_1c*mu_x*(1-e^2)-1/4*beta_1s*mu_y*(1-e^2))+...
        1/4*alpha_1s*(1-e^2)*(beta_1s*mu_x-beta_1c*mu_y) + beta_0*mu_x*(-beta_0p*(1-e^2)+2*(1-e)*(mu_z-lambda_0)-3/2*beta_1c*mu_x*(1-e)+beta_1s*mu_y*(1-e))-...
        1/2*beta_0*beta_1c*mu_y^2*(1-e)+delta/a_0*(1-e^2)*mu_y;

C_YA1s = theta_1s*(1/3*beta_0p*(1-e^3)-1/2*(1-e^2)*(mu_z-lambda_0)-1/8*(1-e^2)*((3*alpha_1s-beta_1c)*mu_x + (alpha_1c+3*beta_1s)*mu_y) - 1/2*(1-e)*beta_0*mu_x*mu_y)+...
        theta_1c*(-1/8*(1-e^2)*((alpha_1c-beta_1s)*mu_x + (alpha_1s+beta_1c)*mu_y) + 1/4*(1-e)*beta_0*(mu_x^2-mu_y^2)) + ...
        theta_0*(-1/3*(1-e^3)*alpha_1s+1/2*(1-e^2)*(beta_0p*mu_x-beta_0*mu_y) - mu_x*(1-e)*(mu_z-lambda_0) + 1/4*(1-e)*beta_1c*(mu_x^2-mu_y^2)-1/2*(1-e)*beta_1s*mu_x*mu_y)+...
        theta_t*(-alpha_1s*(1/4-1/3*e)+1/3*(1-3/2*e+1/2*e^3)*(beta_0p*mu_x-beta_0*mu_y)-1/2*mu_x*(mu_z-lambda_0)*(1+2*e-e^2)+1/8*beta_1c*(mu_x^2-mu_y^2)*(1-e)^2+1/4*beta_1s*mu_x*mu_y*(1-e)^2)+...
        alpha_1s*(2/3*(1-e^3)*beta_0p-(mu_z-lambda_0)*(1-e^2)+3/4*beta_1s*mu_y*(1-e^2)+1/4*beta_1c*mu_x*(1-e^2))+...
        1/4*alpha_1c*(1-e^2)*(beta_1s*mu_x-beta_1c*mu_y) + beta_0*mu_y*(beta_0p*(1-e^2)+2*(1-e)*(mu_z-lambda_0)-3/2*beta_1s*mu_y*(1-e)+beta_1c*mu_x*(1-e))-...
        1/2*beta_0*beta_1s*mu_x^2*(1-e)+delta/a_0*(1-e^2)*mu_x;

% 惯性力系数
Omega_dot = 0;
beta_0pp = 0; beta_1cpp = 0; beta_1spp = 0;
M_bar_beta = M_beta/(rho*pi*R^4);
epsilon_1c = beta_1cp+beta_1s-q_bar_H; epsilon_1s = beta_1sp+beta_1c-p_bar_H;
gamma_1c = beta_1cpp + Omega_dot/(Omega^2)*beta_1s+2*beta_1sp-beta_1c-q_dot_H/Omega^2-p_bar_H;
gamma_1s = beta_1spp + Omega_dot/(Omega^2)*beta_1c-2*beta_1cp-beta_1s-p_dot_H/Omega^2+q_bar_H;

C_XI0 = eta_z*beta_0*m_bar_b + M_bar_beta*(1+beta_0p^2+1/2*(epsilon_1s^2+epsilon_1c^2));
C_XI1c= (eta_x+eta_z*beta_1c)*m_bar_b + 1*beta_0p*epsilon_1c*M_bar_beta;
C_XI1s= -(eta_y-eta_z*beta_1s)*m_bar_b + 1*beta_0p*epsilon_1s*M_bar_beta;

C_YI0 = -M_bar_beta*(beta_0*beta_0p-Omega_dot/Omega^2+1/2*epsilon_1c*(beta_1c-p_bar_H)+1/2*epsilon_1s*(beta_1s+q_bar_H));
C_YI1c = eta_y*m_bar_b-M_bar_beta*(beta_0*epsilon_1c+beta_0p*(beta_1c-p_bar_H));
C_YI1s = eta_x*m_bar_b-M_bar_beta*(beta_0*epsilon_1s+beta_0p*(beta_1s+q_bar_H));

C_ZI0 = -(eta_z+1/2*(eta_y*beta_1s-eta_x*beta_1c))*m_bar_b+M_bar_beta*(beta_0+beta_0pp);
C_ZI1c = eta_x*beta_0*m_bar_b + M_bar_beta*(gamma_1c+beta_1c-p_bar_H);
C_ZI1s = -eta_y*beta_0*m_bar_b + M_bar_beta*(gamma_1s+beta_1c+q_bar_H);

% 合力系数
C_X0 = C_XI0; C_X1c = C_XI1c; C_X1s = C_XI1s;
C_Y1c = s*a_0/(2*b)*C_YA1c+C_YI1c; C_Y1s = s*a_0/(2*b)*C_YA1s+C_YI1s;
C_Z0 = -s*a_0/(2*b)*C_ZA0+C_ZI0; C_Z1c = -s*a_0/(2*b)*C_ZA1c+C_ZI1c; C_Z1s = -s*a_0/(2*b)*C_ZA1s+C_ZI1s;

C_Xh = -b/2*(C_X1c+C_Y1s+beta_0*C_Z1c+beta_1c*C_Z0);
C_Yh = b/2*(C_X1s+C_Y1c+beta_0*C_Z1s+beta_1s*C_Z0);
C_Zh = b/2*(2*C_Z0+2*beta_0*C_Z0+beta_1c*C_X1c-beta_1s*C_X1s);

% 诱导速度升力系数
C_Zhi = 2*lambda_0*sqrt(mu^2+(mu_z-lambda_0)^2);

Z_h = rho*(Omega*R)^2*A*C_Zh;
Z_hi = rho*(Omega*R)^2*A*C_Zhi;

% 合力
X_h = rho*(Omega*R)^2*A*C_Xh;
Y_h = rho*(Omega*R)^2*A*C_Yh;
Z_h = rho*(Omega*R)^2*A*C_Zh;

X_R = X_h*cos(gamma_s) - Z_h*sin(gamma_s);
Y_R = Y_h;
Z_R = X_h*sin(gamma_s) + Z_h*cos(gamma_s);

Y_R = -Y_R;

% 气动力矩系数
C_QA0=theta_1c*(-1/8*(1-e^4)*alpha_1c+1/6*(1-e^3)*(beta_0p*mu_y+beta_0*mu_x)+1/16*(1-e^2)*beta_1s*(mu_x^2-mu_y^2)-1/4*mu_y*(1-e^2)*...
        (mu_z-lambda_0)+1/8*(1-e^2)*beta_1c*mu_x*mu_y)+...
        theta_1s*(-1/8*(1-e^4)*alpha_1s+1/6*(1-e^3)*(beta_0p*mu_x-beta_0*mu_y)+1/16*((1-e^2)*beta_1c*(mu_x^2-mu_y^2))-1/4*mu_x*(1-e^2)*...
        (mu_z-lambda_0)-1/8*(1-e^2)*beta_1s*mu_x*mu_y)+...
        theta_0*(1/4*(1-e^4)*beta_0p-1/3*(1-e^3)*(mu_z-lambda_0)-1/6*(1-e^3)*((alpha_1s-beta_1c)*mu_x+(alpha_1c+beta_1s)*mu_y))+...
        theta_t*((1/5-1/4*e+1/20*e^5)*beta_0p-(1/4-1/3*e+1/12*e^4)*((mu_z-lambda_0)-1/2*((alpha_1s-beta_1c)*mu_x+(alpha_1c+...
        beta_1s)*mu_y)))-...
        1/8*(1-e^4)*(alpha_1c^2+alpha_1s^2+2*beta_0p^2)+(mu_z-lambda_0)*(2/3*beta_0p*(1-e^3)-1/2*(1-e^2)*((mu_z-lambda_0)-(beta_1c*mu_x-...
        beta_1s*mu_y)))+1/3*(1-e^3)*(beta_0*(alpha_1c*mu_x-alpha_1s*mu_y)+beta_0p*(beta_1s*mu_y-beta_1c*mu_x))+(1-e^2)*(-1/4*beta_0^2*mu^2+...
        1/4*beta_1s*beta_1c*mu_x*mu_y-1/16*mu^2*(beta_1s^2+beta_1c^2)-1/8*(beta_1c^2*mu_x^2+beta_1s^2*mu_y^2))+1/4*(1-e^2)*(1+e^2+mu^2)*delta/a_0;
    
C_QA1c = theta_1c*(1/4*beta_0p*(1-e^4)+(1-e^3)*(-1/3*(mu_z-lambda_0)-1/12*(alpha_1s*mu_x+beta_1s*mu_y)+1/4*(beta_1c*mu_x-alpha_1c*mu_y))...
        +1/4*(1-e^2)*beta_0*mu_x*mu_y)+...
        theta_1s*(1/8*(1-e^2)*beta_0*(mu_x^2-mu_y^2)+1/12*((1-e^3)*(mu_x*(beta_1s-alpha_1c)-(alpha_1s+beta_1c)*mu_y)))+...
        theta_0*(-1/4*(1-e^4)*alpha_1c+1/3*(1-e^3)*(beta_0p*mu_y+beta_0*mu_x)+1/2*(1-e^2)*(1/4*beta_1s*(mu_x^2-mu_y^2)-mu_y*(mu_z-lambda_0)...
        +1/2*beta_1c*mu_x*mu_y))+...
        theta_t*(-alpha_1c*(1/5-1/4*e+1/20*e^5)+(1/4-1/3*e+1/12*e^4)*(beta_0p*mu_y+beta_0*mu_x)+1/2*(1-e^3)*(1/2*beta_1s*(mu_x^2-...
        mu_y^2)+beta_1c*mu_x*mu_y-2*mu_y*(mu_z-lambda_0)))+...
        1/2*(1-e^4)*alpha_1c*beta_0p+(1-e^2)*(beta_0*mu_x*(mu_z-lambda_0)-1/2*beta_0*beta_1c*(mu_x^2-1/2*mu^2)+1/2*beta_1s*beta_0*mu_x*mu_y)-...
        2/3*(1-e^3)*(alpha_1c*(mu_z-lambda_0)+beta_0*beta_0p*mu_x-delta*mu_y/a_0+1/4*beta_1s*(alpha_1c*mu_y-alpha_1s*mu_x)+1/4*beta_1c*(alpha_1s*mu_y-...
        3*alpha_1c*mu_x));

C_QA1s = theta_1c*(1/8*(1-e^2)*beta_0*(mu_x^2-mu_y^2)+1/12*(1-e^3)*(mu_x*(beta_1s-alpha_1c)-(alpha_1s+beta_1c)*mu_y))+...
        theta_1s*(1/4*beta_0p*(1-e^4)+(1-e^3)*(-1/3*(mu_z-lambda_0)-1/4*(alpha_1s*mu_x+beta_1s*mu_y)+1/12*(beta_1c*mu_x-alpha_1c*mu_y))...
        -1/4*(1-e^2)*beta_0*mu_x*mu_y)+...
        theta_0*(-1/4*(1-e^4)*alpha_1s+1/3*(1-e^3)*(beta_0p*mu_x-beta_0*mu_y)+1/2*(1-e^2)*(1/4*beta_1c*(mu_x^2-mu_y^2)-mu_x*(mu_z-lambda_0)...
        -1/2*beta_1s*mu_x*mu_y))+...
        theta_t*(-alpha_1s*(1/5-1/4*e+1/20*e^5)+(1/4-1/3*e+1/12*e^4)*(beta_0p*mu_x-beta_0*mu_y)+1/2*(1-e^3)*(1/2*beta_1c*(mu_x^2-...
        mu_y^2)-beta_1s*mu_x*mu_y-2*mu_x*(mu_z-lambda_0)))+...
        1/2*(1-e^4)*alpha_1s*beta_0p+(1-e^2)*(-beta_0*mu_y*(mu_z-lambda_0)-1/2*beta_0*beta_1s*(mu_y^2+1/2*mu^2)+1/2*beta_1c*beta_0*mu_x*mu_y)-...
        2/3*(1-e^3)*(alpha_1s*(mu_z-lambda_0)-beta_0*beta_0p*mu_y-delta*mu_x/a_0+1/4*beta_1s*(alpha_1c*mu_x-3*alpha_1s*mu_y)+1/4*beta_1c*(alpha_1s*mu_x-...
        alpha_1c*mu_y));

% 惯性力矩系数
I_bar_beta = I_beta/(rho*pi*R^5);
C_QI0 = -I_bar_beta*(beta_0*beta_0p-Omega_dot/Omega^2+1/2*epsilon_1c*(beta_1c-p_bar_H)+1/2*epsilon_1s*(beta_1s+q_bar_H));
C_QI1c = eta_y*M_bar_beta - I_bar_beta*(beta_0*epsilon_1c+beta_0p*(beta_1c-p_bar_H));
C_QI1s = eta_x*M_bar_beta - I_bar_beta*(beta_0*epsilon_1s+beta_0p*(beta_1s+q_bar_H));

% 合力矩系数
C_Q0 = s*a_0/(2*b)*C_QA0 + C_QI0;
C_Q1c = s*a_0/(2*b)*C_QA1c + C_QI1c;
C_Q1s = s*a_0/(2*b)*C_QA1s + C_QI1s;

% 合力矩
Q_0 = rho*(Omega*R)^2*A*R*C_Q0;
Q_1c = rho*(Omega*R)^2*A*R*C_Q1c;
Q_1s = rho*(Omega*R)^2*A*R*C_Q1s;

L_h = -b/2*(K_beta*beta_1s+Q_0*beta_1c+Q_1c*beta_0);
M_h = b/2*(beta_1s*Q_0+beta_0*Q_1s-K_beta*beta_1c);
N_h = b*Q_0;

L_R = L_h*cos(gamma_s)-N_h*sin(gamma_s)+h_R2*Y_R;
M_R = M_h-h_R2*X_R+x_cg*Z_R;
N_R = L_h*sin(gamma_s)+N_h*cos(gamma_s)-x_cg*Y_R;

L_R = -L_R;
N_R = -N_R;

F = Z_h+Z_hi;


Rotor.X = X_R;
Rotor.Y = Y_R;
Rotor.Z = Z_R;
Rotor.X_h = X_h;
Rotor.Y_h = Y_h;
Rotor.Z_h = Z_h;
Rotor.L = L_R;
Rotor.M = M_R;
Rotor.N = N_R;
Rotor.v_0 =v_0; 
Rotor.beta_0 = beta_0;
Rotor.beta_1c = beta_1c;
Rotor.beta_1s = beta_1s;
Rotor.v_1c = v_1c;
Rotor.v_1s = v_1s;


end