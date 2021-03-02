function F = AerodynamicsV5(States)
    % 王泽远 1.22
    % 加入舵面偏角和横纵向差动的特性，相应偏角需要在主函数中指定
    
    % 王泽远 1.20
    % 承接V2。无舵面，有尾推，平直平飞/悬停，不含不含部件间气动干扰以及旋翼间干扰
    % 机身，尾翼迎角公式改变
  
    % 王泽远 1.18
    % 无舵面，有尾推，平直平飞/悬停，不含不含部件间气动干扰以及旋翼间干扰
    % 尾推采用曹燕论文拟合数据

    % 王泽远 1.17 
    % 无舵面，无尾推, 平直飞行/悬停，不含部件间气动干扰以及旋翼间干扰
    % 上下旋翼的横纵向变距一样
    % 整机动力学模型 下标1为下，2为上
    % States(1) = v_01
    % States(2) = v_02
    % States(3) = theta_0
    % States(4) = theta_diff
    % States(5) = theta_1c
    % States(6) = theta_1s
    % States(7) = theta_PR
    % States(8) = phi
    % theta 俯仰角为冗余变量，在主程序中预设。调节次数值可以得到不同配平功率
    
    % 王泽远 1.15
    % 1.16 单旋翼下标，单旋翼测试，参数传递。悬停状态测试。
    
    %% 旋翼特性基本测试：模拟旋转塔，无线速度，角速度，横纵变距为0。给定总距，计算受力。
    global Heli Control
    global rho g
    global Rotor b m_b R A Omega s theta_t I_beta a_0 delta c K_beta e M_beta gamma_s h_R1 h_R2 x_cg
    global GW 
    global theta_01 theta_02
    
    %% 操纵变量 delta_e,delta_r, theta_diff, theta_1c1,theta_1s1, theta_1c2,theta_1s2
    delta_e = Control.delta_e;
    delta_r = Control.delta_r;
    % 操纵变量 1c:纵向力 1s：横向力
    theta_01 = States(3)-States(4); 
    theta_1c1=States(5)-Control.theta_1c_diff; 
    theta_1s1=States(6)-Control.theta_1s_diff;
    %操纵变量 1c:纵向力 1s：横向力
    theta_02 = States(3)+States(4); 
    theta_1c2=States(5)+Control.theta_1c_diff; 
    theta_1s2=States(6)+Control.theta_1s_diff;

    %% 运动状态,诱导速度,下旋翼

    theta_0PR = States(7);
    phi = States(8);
    theta = Heli.theta;
    psi = Heli.psi;
    V = Heli.V;
    TBE = [cos(theta)*cos(psi)   cos(theta)*sin(psi)       -sin(theta);
            sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)      sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)      sin(phi)*cos(theta) ;
            cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)      cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)  cos(phi)*cos(theta)];
    uvw = TBE*[Heli.V;0;0];
    Heli.u = uvw(1);
    Heli.v = uvw(2);
    Heli.w = uvw(3);
    
    u = Heli.u; v = Heli.v; w = Heli.w;
    p = Heli.p; q = Heli.q; r = Heli.r;
    u_dot = Heli.u_dot; v_dot = Heli.v_dot; w_dot = Heli.w_dot;
    p_dot = Heli.p_dot; q_dot = Heli.r_dot; r_dot = Heli.r_dot;
    % 运动状态
    u_H1 = (u-q*h_R1)*cos(gamma_s)+(w+q*x_cg*sin(gamma_s));
    v_H1 = v+p*h_R1-r*x_cg;
    w_H1 = (w+q*x_cg)*cos(gamma_s)-(u-q*h_R1)*sin(gamma_s);
    
    p_H1 = p*cos(gamma_s)+r*sin(gamma_s);
    q_H1 = q;
    
    p_dot_H1 = p_dot*cos(gamma_s)+r_dot*sin(gamma_s); 
    q_dot_H1 = q_dot; 
    
    % 诱导速度
    v_01 = States(1); v_1c1 = 0; v_1s1 = 0;
    
    %% 运动状态,诱导速度,上旋翼
    % 运动状态
    u_H2 = (u-q*h_R2)*cos(gamma_s)+(w+q*x_cg*sin(gamma_s));
    v_H2 = -(v+p*h_R2-r*x_cg);
    w_H2 = (w+q*x_cg)*cos(gamma_s)-(u-q*h_R2)*sin(gamma_s);
    
    p_H2 = -(p*cos(gamma_s)+r*sin(gamma_s));
    q_H2 = q;
    
    p_dot_H2 = -(p_dot*cos(gamma_s)+r_dot*sin(gamma_s)); 
    q_dot_H2 = q_dot; 
    
    
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
    mu = V/(Omega*R);
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
    C_Zhi1 = 2*States(1)/(Omega*R)*sqrt(mu_1^2+(mu_z1-lambda_01)^2);
    
    Z_h1 = rho*(Omega*R)^2*A*C_Zh1;
    Z_hi1 = rho*(Omega*R)^2*A*C_Zhi1;
    
    % 合力
    X_h1 = rho*(Omega*R)^2*A*C_Xh1;
    Y_h1 = rho*(Omega*R)^2*A*C_Yh1;
    Z_h1 = rho*(Omega*R)^2*A*C_Zh1;
    
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
    C_Zhi2 = 2*States(2)/(Omega*R)*sqrt(mu_2^2+(mu_z2-lambda_02)^2);
    
    Z_h2 = rho*(Omega*R)^2*A*C_Zh2;
    Z_hi2 = rho*(Omega*R)^2*A*C_Zhi2;
    
    % 合力
    X_h2 = rho*(Omega*R)^2*A*C_Xh2;
    Y_h2 = rho*(Omega*R)^2*A*C_Yh2;
    Z_h2 = rho*(Omega*R)^2*A*C_Zh2;
    
    X_R2 = rho*(Omega*R)^2*A*(C_Xh2*cos(gamma_s) - C_Zh2*sin(gamma_s));
    Y_R2 = rho*(Omega*R)^2*A*C_Yh2;
    Z_R2 = rho*(Omega*R)^2*A*(C_Xh2*sin(gamma_s) + C_Zh2*cos(gamma_s));
    
    Y_R2 = -Y_R2;
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
    % M_R2 = M_R2;
    N_R2 = -N_R2;
    
    % 合力
    X_R = X_R1+X_R2;
    Y_R = Y_R1+Y_R2;
    Z_R = Z_R1+Z_R2;
    L_R = L_R1+L_R2;
    M_R = M_R1+M_R2;
    N_R = N_R1+N_R2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% 机身 X_F Y_F Z_F L_F M_F N_F
    global x_F y_F z_F l_F s_F Fuselage
    
    % chi1=atan(mu_1/(-lambda_1));
    % chi2=atan(mu_2/(-lambda_2));
    % k_1F=1.299+0.671*chi1-1.172*chi1^2+0.351*chi1^3;
    % k_2F=1.299+0.671*chi2-1.172*chi2^2+0.351*chi2^3;
    % w_iF=k_1F*v_1+k_2F*v_2;
    % w_iF=-k_1F*v_1-k_2F*v_2;
    %w_iF=-0.8*v_1-0.8*v_2;
    
    u_F = u + z_F*q - y_F*r;
    v_F = v + x_F*r - z_F*p;
    %w_F = w + y_F*p - x_F*q + w_iF; % 考虑旋翼干扰
    w_F = w + y_F*p - x_F*q; %不考虑旋翼干扰
    
    if u_F == 0
        alpha_F = atan(w_F/1e-10);
    else
        alpha_F = atan(w_F/u_F);
    end
    
    
    if sqrt(u_F^2+v_F^2+w_F^2) == 0
        beta_F = asin(v_F/1e-10);
    else
        beta_F = asin(v_F/sqrt(u_F^2+v_F^2+w_F^2));
    end 
    
    q_F = 1/2*rho*(u_F^2+v_F^2+w_F^2);
    
    % 机身气动系数
    % 升力系数
    xs_alpha_CLF=deg2rad([-10.0986,-7.93095,-6.0119,-4.01,-2.00857,-0.00619048,2.0781,4.08,5.99857,8.08381,10.0024,12.0048,14.0905]);
    CLF=[-0.0291429,-0.0274286,-0.0231429,-0.0197143,-0.0171429,-0.0128571,-0.0111429,-0.00771429,-0.00428571,-8.57E-04,0.00257143,0.00685714,0.0111429];
    if alpha_F < deg2rad(-10.0986) 
            C_LF = 2*-0.0291429;
    elseif alpha_F > deg2rad(14.0905) 
            C_LF = 2*0.0111429;
    else
            C_LF=2*interp1(xs_alpha_CLF,CLF,alpha_F,'spline');
    end
    
    %阻力系数
    xs_CLF=[0.012	0.00942857	0.00514286	0.00171429	-0.00342857	-0.00685714	-0.0102857	-0.0145714	-0.0188571	-0.0231429	-0.0274286];
    CDF=[0.0242429	0.0233952	0.0217048	0.0216857	0.0224905	0.0233048	0.0249524	0.0265952	0.0282381	0.029881	0.0323571];
    C_DF=2*interp1(xs_CLF,CDF,C_LF/2,'spline');
    
    %侧力系数
    xs_beta_CYF=deg2rad([-14.9607	-9.95511	-4.85971	-2.48597	0	2.50281	5	10.0954	15.0842]);
    CYF=[0.0376543	0.0216049	0.00876543	0.00487654	0	-0.00802469	-0.015	-0.0278395	-0.0407407];
    if beta_F < deg2rad(-14.9607)
        C_YF = 2*0.0376543;
    elseif beta_F > deg2rad(15.0842)
        C_YF = 2*-0.0407407;
    else
    C_YF=2*interp1(xs_beta_CYF,CYF,beta_F,'spline');
    end
    
    %俯仰力矩系数
    xs_alpha_CMF=deg2rad([-10.3333	-8.25	-6.25	-4.16667	-2.25	-0.25	1.83333	3.83333	5.75	7.83333	9.75	11.75	13.75]);
    CMF=[-0.0062069	-0.00551724	-0.00448276	-0.0037931	-0.00275862	-0.00103448	-6.90E-04	6.90E-04	0.00172414	0.00275862	0.00344828	0.00448276	0.00517241];
    if alpha_F < deg2rad(-10.3333) 
        C_MF = 2*-0.0062069;
    elseif alpha_F > deg2rad(13.75) 
        C_MF = 2*0.00517241;
    else
    C_MF=2*interp1(xs_alpha_CMF,CMF,alpha_F,'spline');
    end
    
    %偏航力矩系数
    xs_beta_CNF=deg2rad([-14.9164	-9.92281	-4.83368	-2.34242	0	2.64564	5.03582	10.2482	15.1519]);
    CNF=[0.0382407	0.0244349	0.0127673	0.00690287	0.00E+00	-0.00586439	-0.0128283	-0.0275504	-0.0445327];
    if beta_F < deg2rad(-14.9164)
        C_NF = 2*0.0382407;
    elseif beta_F > deg2rad(15.1519)
        C_NF = 2*-0.0445327;
    else
    C_NF=2*interp1(xs_beta_CNF,CNF,beta_F,'spline');
    end
    
    %滚转力矩系数
    xs_beta_CLF=deg2rad([-14.2432	-9.25539	-4.26762	-1.72483	0.72016	3.37853	5.71683	10.9135	15.9013]);
    CRF=[-8.44E-04	-4.79E-04	-1.15E-04	1.10E-04	2.49E-04	3.94E-04	5.31E-04	9.41E-04	0.00130562];
    if beta_F < deg2rad(-14.2432)
        C_RF = 2*-8.44E-04;
    elseif beta_F > deg2rad(15.9013)
        C_RF = 2*0.00130562;
    else
    C_RF=2*interp1(xs_beta_CLF,CRF,beta_F,'spline');
    end
    
    % 受力计算
    FuselageForce=[-q_F*pi*R^2*C_DF;
                   q_F*pi*R^2*C_YF;
                   -q_F*pi*R^2*C_LF];
    X_F=FuselageForce(1);
    Y_F=FuselageForce(2);
    Z_F=FuselageForce(3);
    
    M_fuselage = [q_F*pi*R^2*2*R*C_RF;
                  q_F*pi*R^2*2*R*C_MF;
                  q_F*pi*R^2*2*R*C_NF]+[y_F*Z_F-z_F*Y_F;
                                            z_F*X_F-x_F*Z_F;
                                            x_F*Y_F-y_F*X_F];
    L_F=M_fuselage(1);
    M_F=M_fuselage(2);
    N_F=M_fuselage(3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% 水平尾翼  X_HS Y_HS Z_HS L_HS M_HS N_HS 操纵delta_e,下偏为正
    global s_H s_e xi_H x_HS y_HS z_HS Tail
    % 配平量：delta_e
    % 上下旋翼对平尾垂向速度影响因子
    % k_1HS = -0.1;
    % k_2HS = -0.1;
    % 动压损失系数，大于0小于等于1，越小损失越大
    K_HS = 1; 
    % 主旋翼对平尾下洗干扰速度
    % w_iHS = k_1HS*v_1 + k_2HS*v_2;
    
    u_HS = u + z_HS*q - y_HS*r;
    v_HS = v + x_HS*r - z_HS*p;
    %w_HS = w + y_HS*p - x_HS*q + w_iHS; %包含旋翼干扰
    w_HS = w + y_HS*p - x_HS*q; %不含旋翼干扰
    
    % 动压
    
    q_HS = 1/2*K_HS*rho*(u_HS^2+v_HS^2+w_HS^2);
   
    % alpha_0F为机身对平尾下洗
    alpha_0F = 0;
    %alpha_HS = alpha_0F + alpha_0HS + atan(w_HS/u_HS); 
    if u_HS == 0
        alpha_HS = alpha_0F +  atan(w_HS/1e-10);  
        beta_HS = asin(v_HS/1e-10);
    else
        alpha_HS = alpha_0F +  atan(w_HS/u_HS);  
        beta_HS = asin(v_HS/sqrt(u_HS^2+v_HS^2+w_HS^2));
    end
    
    % 升降舵效率因子
    k_e = sqrt(s_e/s_H)*cos(xi_H);
    % 升降舵升力
    % L_e = q_HS*s_H*C_XHFitted(alpha_HS,alpha_F)*delta_e*k_e;
    L_e = q_HS*s_H*5.7*delta_e*k_e;
    
    % 平尾气动力，力矩
    C_LHS=5.7*alpha_HS;        % C_LHS=-3.5*alpha_HS;
    
    C_DHS=0.008;
    %不考虑升降舵
    F_horizontalstabilizer=[cos(beta_HS)*cos(alpha_HS),-sin(beta_HS)*cos(alpha_HS),-sin(alpha_HS);
                                        sin(beta_HS),                       cos(beta_HS),                       0;
                                        cos(beta_HS)*sin(alpha_HS),-sin(beta_HS)*sin(alpha_HS),cos(alpha_HS)]*...
                                [-q_HS*s_H*C_DHS;   
                                0;  
                                -q_HS*s_H*C_LHS+L_e];
    X_HS=F_horizontalstabilizer(1);
    Y_HS=F_horizontalstabilizer(2);
    Z_HS=F_horizontalstabilizer(3);
    
    M_horizontalstabilizer=[y_HS*Z_HS-z_HS*Y_HS;z_HS*X_HS-x_HS*Z_HS;x_HS*Y_HS-y_HS*X_HS];
    L_HS=M_horizontalstabilizer(1);
    M_HS=M_horizontalstabilizer(2);
    N_HS=M_horizontalstabilizer(3);
    
    %% 垂直尾翼 X_VS Y_VS Z_VS L_VS M_VS N_VS 操纵delta_r,右偏为正
    global x_VS y_VS z_VS s_r xi_V s_V Fin
    % 上下旋翼对垂尾垂向速度影响因子
    % k_1VS = -1;
    % k_2VS = -1;
    % 动压损失系数，大于0小于等于1，越小损失越大
    K_VS = 1; 
    % 主旋翼对平尾下洗干扰速度
    % w_iVS = k_1VS*v_1 + k_2VS*v_2;
    
    u_VS = u + z_VS*q - y_VS*r;
    v_VS = v + x_VS*r - z_VS*p;
    %w_VS = w + y_VS*p - x_VS*q + w_iVS; %包含旋翼干扰
    w_VS = w + y_VS*p - x_VS*q; %不包含旋翼干扰
    
    % 动压
    q_VS = 1/2*K_VS*rho*(u_VS^2+v_VS^2+w_VS^2);
    
    if u_VS == 0
        alpha_VS = 0;
    else
        alpha_VS = atan(v_VS/u_VS); 
    end
    
    if sqrt(u_VS^2+v_VS^2+w_VS^2) == 0
        beta_VS = 0;
    else
        beta_VS = asin(w_VS/sqrt(u_VS^2+v_VS^2+w_VS^2));
    end
    
    % 方向舵效率因子
    k_r = sqrt(s_r/s_V)*cos(xi_V);
    % 方向舵升力
    L_r = -q_VS*s_V*5.7*delta_r*k_r;
    
    % 垂尾气动力，力矩
    C_LVS=5.7*alpha_VS;         % C_LVS=-3.5*beta_VS;
    C_DVS=0.008;
    
    F_verticalstabilizer=[cos(beta_VS)*cos(alpha_VS),-sin(alpha_VS),  -sin(beta_VS)*cos(alpha_VS);
                                cos(beta_VS)*sin(alpha_VS),    cos(alpha_VS), -sin(beta_VS)*sin(alpha_VS);
                                   sin(beta_VS),                            0,                      cos(beta_VS)]*...
                                [-q_VS*s_V*C_DVS;
                                 -q_VS*s_V*C_LVS+L_r;
                                  0];
    X_VS=F_verticalstabilizer(1);
    Y_VS=F_verticalstabilizer(2);
    Z_VS=F_verticalstabilizer(3);
    
    M_verticalstabilizer=[y_VS*Z_VS-z_VS*Y_VS;z_VS*X_VS-x_VS*Z_VS;x_VS*Y_VS-y_VS*X_VS];
    L_VS=M_verticalstabilizer(1);
    M_VS=M_verticalstabilizer(2);
    N_VS=M_verticalstabilizer(3);

    %% 尾推螺旋桨 NUAA
    global Propeller x_PR y_PR z_PR Omega_PR R_PR
    theta_0PR_list = deg2rad([4	8	12	16	20	24	28	32	36])';
    X_PR_list = [291	873	1673	2327	3055	4000	5455	8146	10909]';
    L_PR_list = [318.18	545.45	909.09	1272.7	1909.1	2546.5	3363.6	4477.3	6954.5]';
    X_PRFitted = fit(theta_0PR_list, X_PR_list, 'linearinterp');
    L_PRFitted = fit(theta_0PR_list, L_PR_list, 'linearinterp');

    X_PR = X_PRFitted(theta_0PR);
    Y_PR = 0;
    Z_PR = 0;

    L_PR = L_PRFitted(theta_0PR) + (y_PR*Z_PR-z_PR*Y_PR);
    M_PR = z_PR*X_PR-x_PR*Z_PR;
    N_PR = x_PR*Y_PR-y_PR*X_PR;
    
    % 运动状态
    u_PR = u + q*z_PR - r*y_PR;
    v_PR = v + r*x_PR - p*z_PR;
    w_PR = w + p*y_PR - q*x_PR;
    mu_xPR =  u_PR/(Omega_PR*R_PR);
    mu_yPR =  v_PR/(Omega_PR*R_PR);
    mu_zPR =  w_PR/(Omega_PR*R_PR);
    mu_PR = sqrt(mu_xPR^2+mu_yPR^2+mu_zPR^2);
    
    C_TPR = X_PR/((Omega_PR*R_PR)^2*(pi*R_PR^2));
    lambda_0PR = fsolve(@(l0PR) C_TPR - 2*l0PR*sqrt(mu_yPR^2+mu_zPR^2+(l0PR+mu_xPR)^2),10,optimset('Display','off'));
    v_0PR = lambda_0PR*(Omega_PR*R_PR);
        
    %% 方程构建%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    F(1) = Z_h1 + Z_hi1;
    F(2) = Z_h2 + Z_hi2;
    F(3) = X_R + X_F + X_HS + X_VS + X_PR - GW*g*sin(theta);
    F(4) = Y_R + Y_F + Y_HS + Y_VS + Y_PR + GW*g*cos(theta)*sin(phi);
    F(5) = Z_R + Z_F + Z_HS + Z_VS + Z_PR + GW*g*cos(theta)*cos(phi);
    F(6) = L_R + L_F + L_HS + L_VS + L_PR;
    F(7) = M_R + M_F + M_HS + M_VS + M_VS;
    F(8) = N_R + N_F + N_HS + N_VS + N_VS;
    
    %% 储存数据
    Rotor.X_R1 = X_R1;
    Rotor.Y_R1 = Y_R1;
    Rotor.Z_R1 = Z_R1;
    Rotor.X_h1 = X_h1;
    Rotor.Y_h1 = Y_h1;
    Rotor.Z_h1 = Z_h1;
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
    Rotor.X_h2 = X_h2;
    Rotor.Y_h2 = Y_h2;
    Rotor.Z_h2 = Z_h2;
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
    Rotor.theta_0 = (theta_02+theta_01)/2;
    Rotor.theta_diff = (theta_02-theta_01)/2;
    Rotor.X_R = X_R;
    Rotor.Y_R = Y_R;
    Rotor.Z_R = Z_R;
    Rotor.L_R = L_R;
    Rotor.M_R = M_R;
    Rotor.N_R = N_R;
    
    Fuselage.X_F = X_F;
    Fuselage.Y_F = Y_F;
    Fuselage.Z_F = Z_F;
    Fuselage.L_F = L_F;
    Fuselage.M_F = M_F;
    Fuselage.N_F = N_F;
    
    Tail.X_HS = X_HS;
    Tail.Y_HS = Y_HS;
    Tail.Z_HS = Z_HS;
    Tail.L_HS = L_HS;
    Tail.M_HS = M_HS;
    Tail.N_HS = N_HS;
    Fin.X_VS = X_VS;
    Fin.Y_VS = Y_VS;
    Fin.Z_VS = Z_VS;
    Fin.L_VS = L_VS;
    Fin.M_VS = M_VS;
    Fin.N_VS = N_VS;

    Propeller.X_PR = X_PR;
    Propeller.Y_PR = Y_PR;
    Propeller.Z_PR = Z_PR;
    Propeller.L_PR = L_PR;
    Propeller.M_PR = M_PR;
    Propeller.N_PR = N_PR;
    Propeller.v_0PR = v_0PR;
    Propeller.mu_PR = mu_PR;
    
    Heli.mu = mu;
    Heli.theta = theta;
    Heli.phi = phi;
    Heli.X = F(3);
    Heli.Y = F(4);
    Heli.Z = F(5);
    Heli.L = F(6);
    Heli.M = F(7);
    Heli.N = F(8);

    Control.theta_0 = (theta_02+theta_01)/2;
    Control.theta_diff = (theta_02-theta_01)/2;
    Control.theta_1c1 = theta_1c1;
    Control.theta_1s1 = theta_1s1;
    Control.theta_1c2 = theta_1c2;
    Control.theta_1s2 = theta_1s2;
    Control.theta_0PR = theta_0PR;
    %
    end