function [beta_0,beta_1s,beta_1c] = Flapping2(u,v,w,theta_0,theta_1c,theta_1s,v_0)
global rho
global R Omega theta_t I_beta a_0 c K_beta e M_beta gamma_s 

gamma_M =rho*a_0*c*R^4/I_beta;
epsilon = e;
mu_1 = sqrt(u^2+v^2+w^2)/(Omega*R);

u_H=u; v_H=v; w_H=w;
u_dot=0; v_dot=0; w_dot=0;
p=0; q=0; r=0;
p_H=0; q_H=0;
p_dot=0; q_dot=0; r_dot=0;
mu_x = u_H/(Omega*R); mu_y = v_H/(Omega*R); mu_z = w_H/(Omega*R); mu = sqrt(mu_x^2+mu_y^2);
q_bar_H = q_H/Omega; p_bar_H = p_H/Omega;
lambda_0 = v_0/(Omega*R);  
a_Hx = (u_dot-(q_dot+p*r)+(q^2+r^2))*cos(gamma_s)+(w_dot+(p^2+q^2)+(q_dot-p*r)*sin(gamma_s));
a_Hy = v_dot+(p_dot-q*r)-(r_dot+p*q);
a_Hz = (w_dot+(p^2+q^2)+(q_dot-p*r))*cos(gamma_s)-(u_dot-(q_dot+p*r)+(q^2+r^2))*sin(gamma_s);
eta_x = a_Hx/(Omega^2*R); eta_y = a_Hy/(Omega^2*R); eta_z = a_Hz/(Omega^2*R);

if mu_x == 0
    phi_W = pi/2*sign(mu_y);
elseif mu_x < 0 && mu_y == 0
    phi_W = pi;
else
    phi_W = atan(mu_y/mu_x);
end

Chi_W = atan(mu/(lambda_0-mu_z));   

if Chi_W < pi/2
    lambda_1cW = lambda_0*tan(Chi_W/2);
else
    lambda_1cW = lambda_0*cot(Chi_W/2);
end
    
lambda_1c =  lambda_1cW;
lambda_1s =  0;
%% 旋翼挥舞补充方程
% 旋翼挥舞方程，计算旋翼的锥度角a0，横纵向挥舞角a0,b0,控制量

% 下旋翼桨盘运动阻尼矩阵 D_1
% D_1 = Omega* [gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon)            0           -gamma_M*mu_1/4*(1/3-epsilon+epsilon^2);
%                0                            gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2)                             2;
%                -gamma_M*mu_1*(1/3-epsilon+epsilon^2)              -2          gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2)];

% 下桨盘刚度矩阵 k_1
k_1 = zeros(3,3);
k_1(1,1) = 1 + K_beta/(I_beta*Omega^2) + epsilon*M_beta/I_beta;
k_1(1,2) = -gamma_M*mu_1/4*(1/2*epsilon-epsilon^2);
k_1(1,3) = 0;
k_1(2,1) = -gamma_M*mu_1/2*(1/3-1/2*epsilon);
k_1(2,2) = K_beta/(I_beta*Omega^2) + epsilon*M_beta/I_beta;
k_1(2,3)  = gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma_M*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
k_1(3,1) = 0;
k_1(3,2) = -gamma_M/2*(1/4-2/3*epsilon+1/2*epsilon^2) + gamma_M*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
k_1(3,3) = K_beta/(I_beta*Omega^2) + epsilon*M_beta/I_beta;


% 下旋翼外激励项 f1_1
f1_1 = zeros(3,4);
f1_1(1,1) = gamma_M/2*(1/4-1/3*epsilon)+gamma_M*mu_1^2/4*(1/2-epsilon+1/2*epsilon^2);
f1_1(1,2) = gamma_M/2*(1/5-1/4*epsilon)+gamma_M*mu_1^2/4*(1/3-1/2*epsilon);
f1_1(1,4) = -gamma_M*mu_1^2/2*(1/3-1/2*epsilon);
f1_1(2,3) = gamma_M/2*(1/4-1/3*epsilon) + gamma_M*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);
f1_1(3,1) = -gamma_M*mu_1/2*(2/3-epsilon);
f1_1(3,2) = -gamma_M*mu_1/2*(1/2-2/3*epsilon);
f1_1(3,4) = gamma_M/2*(1/4-1/3*epsilon)+3*gamma_M*mu_1^2/8*(1/2-epsilon+1/2*epsilon^2);


% 下旋翼外激励项 f2_1
f2_1 = zeros(3,4);
f2_1(1,1) = gamma_M*mu_1/(8*Omega)*(2/3-epsilon);
f2_1(2,1) = -2*(1+epsilon*M_beta/I_beta);
f2_1(2,2) = -gamma_M/2*(1/4-1/3*epsilon);
f2_1(2,4) = -1;
f2_1(3,1) = -gamma_M/2*(1/4-1/3*epsilon);
f2_1(3,2) = 2*(1+epsilon*M_beta/I_beta);
f2_1(3,3) = -1;

% 下旋翼外激励项 f3_1
f3_1 = zeros(3,3);
f3_1(1,1) = gamma_M/2*(1/3-1/2*epsilon);
f3_1(1,3) = gamma_M*mu_1/8*(2/3-epsilon);
f3_1(2,2) = -gamma_M/2*(1/4-epsilon/3);
f3_1(3,1) = -gamma_M*mu_1/2*(1/2-epsilon+1/2*epsilon^2);
f3_1(3,3) = -gamma_M/2*(1/4-epsilon/3);

% 下旋翼外激励项 f4_1
f4_1 = [M_beta/I_beta;0;0];

% 下旋翼合外激励 f1
f1 = f1_1*[theta_0; theta_t; theta_1c; theta_1s]+f2_1*[0;0;0;0] + f3_1*[-lambda_0;-lambda_1s;-lambda_1c] + f4_1;

Flapping = inv(k_1)*f1;
beta_0 = Flapping(1);
a1s = Flapping(2);
a1c = Flapping(3);

beta1s = a1s*cos(phi_W)-a1c*sin(phi_W);
beta1c = a1c*cos(phi_W)+a1s*sin(phi_W);

beta_1s=-beta1s;
beta_1c=-beta1c;

end
