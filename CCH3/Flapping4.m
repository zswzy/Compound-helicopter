function [beta_0,beta_1c,beta_1s,beta_1cW,beta_1sW] = Flapping4(u,v,w,theta_0,theta_1c,theta_1s,v_0)
global rho
global R Omega theta_t I_beta a_0 c K_beta e M_beta gamma_s 

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

% 挥舞方程
n_beta = rho*c*a_0*R^4/(8*I_beta);
q_Hp = q_bar_H/Omega^2; p_Hp = p_bar_H/Omega^2;
lambda_beta = sqrt(1+K_beta/(I_beta*Omega^2));

% 转换
mu_x = mu;
mu_y = 0;

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
beta_1cW = flap(2);
beta_1sW = flap(3);

beta_1s = beta_1sW*cos(phi_W)-beta_1cW*sin(phi_W);
beta_1c = beta_1cW*cos(phi_W)+beta_1sW*sin(phi_W);


end
