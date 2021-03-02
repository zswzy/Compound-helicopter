% 2.7
clear all
clc
h = 10;
[~,~,~,rho] = atmosisa(h);

SingleRotorHelicopter = Helicopter();

%% private 属性设置
SingleRotor = RotorFixed('anticlockwise');
SingleRotor.a_0        = 5.7;           % 主旋翼升力线斜率,NACA0012
SingleRotor.b          = 3;             % 旋翼桨叶数
SingleRotor.c          = 0.29;          % 主旋翼桨叶弦长m
SingleRotor.delta      = 0.008;         % 主旋翼桨叶阻力系数
SingleRotor.e          = 2.58/5.49;     % 无量纲等效铰偏置量
SingleRotor.e_oswald   = 0.8;           % 奥斯瓦尔德效率因子 (0.8)
SingleRotor.gamma_s    = deg2rad(0);    % 主旋翼桨毂纵向安装角
SingleRotor.h_R        = 0.89;          % 主旋翼处于重心之上的位置
SingleRotor.m_b        = 60;            % 桨叶质量
SingleRotor.omega_n    = 1.4*35.9;      % 主旋翼一阶挥舞固有频率 
SingleRotor.rho        = rho;           % 空气密度
SingleRotor.s          = 0.127;         % 主旋翼实度
SingleRotor.theta_t    = deg2rad(-10);  % 主旋翼桨叶扭转角 rad
SingleRotor.x_cg       = 0;             % 主旋翼处于重心之后的位置
SingleRotor.x_H        = 0;             % 桨毂x位置
SingleRotor.y_H        = 0;             % 桨毂y位置
SingleRotor.z_H        = -0.89;         % 桨毂z位置
SingleRotor.z_diff     = 0.77;          % 上下旋翼间距 m
SingleRotor.I_beta     = 450;           % 主旋翼挥舞惯性矩 kg m^2
SingleRotor.K_beta     = 183397;        % K_beta 弹簧刚度  Nm/rad
SingleRotor.M_beta     = 123;           % 主旋翼对挥舞铰的质量静矩
SingleRotor.Omega      = 35.9;          % 主旋翼转速rad/s
SingleRotor.R          = 5.49;          % 主旋翼半径 m


%% 实例1：悬停
SingleRotorHelicopter.u         = 0;
SingleRotorHelicopter.v         = 0;
SingleRotorHelicopter.w         = 0;
SingleRotorHelicopter.u_dot     = 0;
SingleRotorHelicopter.v_dot     = 0;
SingleRotorHelicopter.w_dot     = 0;
SingleRotorHelicopter.p         = 0;
SingleRotorHelicopter.q         = 0;
SingleRotorHelicopter.r         = 0;
SingleRotorHelicopter.p_dot     = 0;
SingleRotorHelicopter.q_dot     = 0;
SingleRotorHelicopter.r_dot     = 0;

SingleRotor.u        = SingleRotorHelicopter.u;
SingleRotor.v        = SingleRotorHelicopter.v;
SingleRotor.w        = SingleRotorHelicopter.w;
SingleRotor.u_dot    = SingleRotorHelicopter.u_dot;
SingleRotor.v_dot    = SingleRotorHelicopter.v_dot;
SingleRotor.w_dot    = SingleRotorHelicopter.w_dot;
SingleRotor.p        = SingleRotorHelicopter.p;
SingleRotor.q        = SingleRotorHelicopter.q;
SingleRotor.r        = SingleRotorHelicopter.r;
SingleRotor.p_dot    = SingleRotorHelicopter.p_dot;
SingleRotor.q_dot    = SingleRotorHelicopter.q_dot;
SingleRotor.r_dot    = SingleRotorHelicopter.r_dot;

SingleRotor.v_0          = 10;
SingleRotor.theta_0      = deg2rad(15);
SingleRotor.theta_1s     = deg2rad(0);
SingleRotor.theta_1c     = deg2rad(0);
%SingleRotor.calculate_flapping_angle();
SingleRotor.calculate_flapping_angle_3blades();
SingleRotor.calculate_force();
SingleRotor.calculate_torque();

%% 实例2：前飞
SingleRotorHelicopter.u         = 10;
SingleRotorHelicopter.v         = 0;
SingleRotorHelicopter.w         = 0;
SingleRotorHelicopter.u_dot     = 0;
SingleRotorHelicopter.v_dot     = 0;
SingleRotorHelicopter.w_dot     = 0;
SingleRotorHelicopter.p         = 0;
SingleRotorHelicopter.q         = 0;
SingleRotorHelicopter.r         = 0;
SingleRotorHelicopter.p_dot     = 0;
SingleRotorHelicopter.q_dot     = 0;
SingleRotorHelicopter.r_dot     = 0;

SingleRotor.u        = SingleRotorHelicopter.u;
SingleRotor.v        = SingleRotorHelicopter.v;
SingleRotor.w        = SingleRotorHelicopter.w;
SingleRotor.u_dot    = SingleRotorHelicopter.u_dot;
SingleRotor.v_dot    = SingleRotorHelicopter.v_dot;
SingleRotor.w_dot    = SingleRotorHelicopter.w_dot;
SingleRotor.p        = SingleRotorHelicopter.p;
SingleRotor.q        = SingleRotorHelicopter.q;
SingleRotor.r        = SingleRotorHelicopter.r;
SingleRotor.p_dot    = SingleRotorHelicopter.p_dot;
SingleRotor.q_dot    = SingleRotorHelicopter.q_dot;
SingleRotor.r_dot    = SingleRotorHelicopter.r_dot;

SingleRotor.v_0          = 10;
SingleRotor.theta_0      = deg2rad(15);
SingleRotor.theta_1s     = deg2rad(0);
SingleRotor.theta_1c     = deg2rad(0);
%SingleRotor.calculate_flapping_angle();
SingleRotor.calculate_flapping_angle_3blades();
SingleRotor.calculate_force();
SingleRotor.calculate_torque();

%% 实例2：倒飞
SingleRotorHelicopter.u         = -10;
SingleRotorHelicopter.v         = 0;
SingleRotorHelicopter.w         = 0;
SingleRotorHelicopter.u_dot     = 0;
SingleRotorHelicopter.v_dot     = 0;
SingleRotorHelicopter.w_dot     = 0;
SingleRotorHelicopter.p         = 0;
SingleRotorHelicopter.q         = 0;
SingleRotorHelicopter.r         = 0;
SingleRotorHelicopter.p_dot     = 0;
SingleRotorHelicopter.q_dot     = 0;
SingleRotorHelicopter.r_dot     = 0;

SingleRotor.u        = SingleRotorHelicopter.u;
SingleRotor.v        = SingleRotorHelicopter.v;
SingleRotor.w        = SingleRotorHelicopter.w;
SingleRotor.u_dot    = SingleRotorHelicopter.u_dot;
SingleRotor.v_dot    = SingleRotorHelicopter.v_dot;
SingleRotor.w_dot    = SingleRotorHelicopter.w_dot;
SingleRotor.p        = SingleRotorHelicopter.p;
SingleRotor.q        = SingleRotorHelicopter.q;
SingleRotor.r        = SingleRotorHelicopter.r;
SingleRotor.p_dot    = SingleRotorHelicopter.p_dot;
SingleRotor.q_dot    = SingleRotorHelicopter.q_dot;
SingleRotor.r_dot    = SingleRotorHelicopter.r_dot;

SingleRotor.v_0          = 10;
SingleRotor.theta_0      = deg2rad(15);
SingleRotor.theta_1s     = deg2rad(0);
SingleRotor.theta_1c     = deg2rad(0);
%SingleRotor.calculate_flapping_angle();
SingleRotor.calculate_flapping_angle_3blades();
SingleRotor.calculate_force();
SingleRotor.calculate_torque();

%% 实例2：右飞
SingleRotorHelicopter.u         = 0;
SingleRotorHelicopter.v         = 10;
SingleRotorHelicopter.w         = 0;
SingleRotorHelicopter.u_dot     = 0;
SingleRotorHelicopter.v_dot     = 0;
SingleRotorHelicopter.w_dot     = 0;
SingleRotorHelicopter.p         = 0;
SingleRotorHelicopter.q         = 0;
SingleRotorHelicopter.r         = 0;
SingleRotorHelicopter.p_dot     = 0;
SingleRotorHelicopter.q_dot     = 0;
SingleRotorHelicopter.r_dot     = 0;

SingleRotor.u        = SingleRotorHelicopter.u;
SingleRotor.v        = SingleRotorHelicopter.v;
SingleRotor.w        = SingleRotorHelicopter.w;
SingleRotor.u_dot    = SingleRotorHelicopter.u_dot;
SingleRotor.v_dot    = SingleRotorHelicopter.v_dot;
SingleRotor.w_dot    = SingleRotorHelicopter.w_dot;
SingleRotor.p        = SingleRotorHelicopter.p;
SingleRotor.q        = SingleRotorHelicopter.q;
SingleRotor.r        = SingleRotorHelicopter.r;
SingleRotor.p_dot    = SingleRotorHelicopter.p_dot;
SingleRotor.q_dot    = SingleRotorHelicopter.q_dot;
SingleRotor.r_dot    = SingleRotorHelicopter.r_dot;

SingleRotor.v_0          = 10;
SingleRotor.theta_0      = deg2rad(15);
SingleRotor.theta_1s     = deg2rad(0);
SingleRotor.theta_1c     = deg2rad(0);
%SingleRotor.calculate_flapping_angle();
SingleRotor.calculate_flapping_angle_3blades();
SingleRotor.calculate_force();
SingleRotor.calculate_torque();

%% 实例2：左飞
SingleRotorHelicopter.u         = 0;
SingleRotorHelicopter.v         = -10;
SingleRotorHelicopter.w         = 0;
SingleRotorHelicopter.u_dot     = 0;
SingleRotorHelicopter.v_dot     = 0;
SingleRotorHelicopter.w_dot     = 0;
SingleRotorHelicopter.p         = 0;
SingleRotorHelicopter.q         = 0;
SingleRotorHelicopter.r         = 0;
SingleRotorHelicopter.p_dot     = 0;
SingleRotorHelicopter.q_dot     = 0;
SingleRotorHelicopter.r_dot     = 0;

SingleRotor.u        = SingleRotorHelicopter.u;
SingleRotor.v        = SingleRotorHelicopter.v;
SingleRotor.w        = SingleRotorHelicopter.w;
SingleRotor.u_dot    = SingleRotorHelicopter.u_dot;
SingleRotor.v_dot    = SingleRotorHelicopter.v_dot;
SingleRotor.w_dot    = SingleRotorHelicopter.w_dot;
SingleRotor.p        = SingleRotorHelicopter.p;
SingleRotor.q        = SingleRotorHelicopter.q;
SingleRotor.r        = SingleRotorHelicopter.r;
SingleRotor.p_dot    = SingleRotorHelicopter.p_dot;
SingleRotor.q_dot    = SingleRotorHelicopter.q_dot;
SingleRotor.r_dot    = SingleRotorHelicopter.r_dot;

SingleRotor.v_0          = 10;
SingleRotor.theta_0      = deg2rad(10);
SingleRotor.theta_1s     = deg2rad(0);
SingleRotor.theta_1c     = deg2rad(0);
%SingleRotor.calculate_flapping_angle();
SingleRotor.calculate_flapping_angle_3blades();
SingleRotor.calculate_force();
SingleRotor.calculate_torque();

%% 右前飞
SingleRotorHelicopter.u         = 10;
SingleRotorHelicopter.v         = 10;
SingleRotorHelicopter.w         = 0;
SingleRotorHelicopter.u_dot     = 0;
SingleRotorHelicopter.v_dot     = 0;
SingleRotorHelicopter.w_dot     = 0;
SingleRotorHelicopter.p         = 0;
SingleRotorHelicopter.q         = 0;
SingleRotorHelicopter.r         = 0;
SingleRotorHelicopter.p_dot     = 0;
SingleRotorHelicopter.q_dot     = 0;
SingleRotorHelicopter.r_dot     = 0;

SingleRotor.u        = SingleRotorHelicopter.u;
SingleRotor.v        = SingleRotorHelicopter.v;
SingleRotor.w        = SingleRotorHelicopter.w;
SingleRotor.u_dot    = SingleRotorHelicopter.u_dot;
SingleRotor.v_dot    = SingleRotorHelicopter.v_dot;
SingleRotor.w_dot    = SingleRotorHelicopter.w_dot;
SingleRotor.p        = SingleRotorHelicopter.p;
SingleRotor.q        = SingleRotorHelicopter.q;
SingleRotor.r        = SingleRotorHelicopter.r;
SingleRotor.p_dot    = SingleRotorHelicopter.p_dot;
SingleRotor.q_dot    = SingleRotorHelicopter.q_dot;
SingleRotor.r_dot    = SingleRotorHelicopter.r_dot;

SingleRotor.v_0          = 10;
SingleRotor.theta_0      = deg2rad(15);
SingleRotor.theta_1s     = deg2rad(0);
SingleRotor.theta_1c     = deg2rad(0);
%SingleRotor.calculate_flapping_angle();
SingleRotor.calculate_flapping_angle_3blades();
SingleRotor.calculate_force();
SingleRotor.calculate_torque();

%% 左后飞
SingleRotorHelicopter.u         = -10;
SingleRotorHelicopter.v         = -10;
SingleRotorHelicopter.w         = 0;
SingleRotorHelicopter.u_dot     = 0;
SingleRotorHelicopter.v_dot     = 0;
SingleRotorHelicopter.w_dot     = 0;
SingleRotorHelicopter.p         = 0;
SingleRotorHelicopter.q         = 0;
SingleRotorHelicopter.r         = 0;
SingleRotorHelicopter.p_dot     = 0;
SingleRotorHelicopter.q_dot     = 0;
SingleRotorHelicopter.r_dot     = 0;

SingleRotor.u        = SingleRotorHelicopter.u;
SingleRotor.v        = SingleRotorHelicopter.v;
SingleRotor.w        = SingleRotorHelicopter.w;
SingleRotor.u_dot    = SingleRotorHelicopter.u_dot;
SingleRotor.v_dot    = SingleRotorHelicopter.v_dot;
SingleRotor.w_dot    = SingleRotorHelicopter.w_dot;
SingleRotor.p        = SingleRotorHelicopter.p;
SingleRotor.q        = SingleRotorHelicopter.q;
SingleRotor.r        = SingleRotorHelicopter.r;
SingleRotor.p_dot    = SingleRotorHelicopter.p_dot;
SingleRotor.q_dot    = SingleRotorHelicopter.q_dot;
SingleRotor.r_dot    = SingleRotorHelicopter.r_dot;

SingleRotor.u_dot_H      = 0;
SingleRotor.v_dot_H      = 0;
SingleRotor.w_dot_H      = 0;

SingleRotor.v_0          = 10;
SingleRotor.theta_0      = deg2rad(15);
SingleRotor.theta_1s     = deg2rad(0);
SingleRotor.theta_1c     = deg2rad(0);
%SingleRotor.calculate_flapping_angle();
SingleRotor.calculate_flapping_angle_3blades();
SingleRotor.calculate_force();
SingleRotor.calculate_torque();

%% test 2.8
SingleRotor.u = 0;
SingleRotor.v = 10;
SingleRotor.beta_0 = 0.0197;
SingleRotor.beta_1c = -0.005;
SingleRotor.beta_1s = 0.01;
SingleRotor.calculate_force();

%% test 2.10
SingleRotor.u = 0;
SingleRotor.v = 10;
SingleRotor.lambda_1c = 0;
SingleRotor.lambda_1s = -0.021016;
SingleRotor.calculate_flapping_angle();
