% 2.12 上下旋翼镜像力调试
clear all
clc
h = 10;
[~,~,~,rho] = atmosisa(h);

DoubleRotorHelicopter = Helicopter();

%% private 属性设置
LowerRotor = RotorFixed('anticlockwise'); % 下旋翼逆时针转， 为默认情况
LowerRotor.a_0        = 5.7;           % 主旋翼升力线斜率,NACA0012
LowerRotor.b          = 3;             % 旋翼桨叶数
LowerRotor.c          = 0.29;          % 主旋翼桨叶弦长m
LowerRotor.delta      = 0.008;         % 主旋翼桨叶阻力系数
LowerRotor.e          = 2.58/5.49;     % 无量纲等效铰偏置量
LowerRotor.gamma_s    = deg2rad(0);    % 主旋翼桨毂纵向安装角
LowerRotor.h_R        = 0.89;          % 主旋翼处于重心之上的位置
LowerRotor.m_b        = 60;            % 桨叶质量
LowerRotor.omega_n    = 1.4*35.9;      % 主旋翼一阶挥舞固有频率 
LowerRotor.rho        = rho;           % 空气密度
LowerRotor.s          = 0.127;         % 主旋翼实度
LowerRotor.theta_t    = deg2rad(-10);  % 主旋翼桨叶扭转角 rad
LowerRotor.x_cg       = 0;             % 主旋翼处于重心之后的位置
LowerRotor.x_H        = 0;             % 桨毂x位置
LowerRotor.y_H        = 0;             % 桨毂y位置
LowerRotor.z_H        = -0.89;         % 桨毂z位置
LowerRotor.z_diff     = 0.77;          % 上下旋翼间距 m
LowerRotor.I_beta     = 450;           % 主旋翼挥舞惯性矩 kg m^2
LowerRotor.K_beta     = 183397;        % K_beta 弹簧刚度  Nm/rad
LowerRotor.M_beta     = 123;           % 主旋翼对挥舞铰的质量静矩
LowerRotor.Omega      = 35.9;          % 主旋翼转速rad/s
LowerRotor.R          = 5.49;          % 主旋翼半径 m

UpperRotor = RotorFixed('clockwise'); % 上旋翼顺时针转
UpperRotor.a_0        = 5.7;           % 主旋翼升力线斜率,NACA0012
UpperRotor.b          = 3;             % 旋翼桨叶数
UpperRotor.c          = 0.29;          % 主旋翼桨叶弦长m
UpperRotor.delta      = 0.008;         % 主旋翼桨叶阻力系数
UpperRotor.e          = 2.58/5.49;     % 无量纲等效铰偏置量
UpperRotor.gamma_s    = deg2rad(0);    % 主旋翼桨毂纵向安装角
UpperRotor.h_R        = 0.89;          % 主旋翼处于重心之上的位置
UpperRotor.m_b        = 60;            % 桨叶质量
UpperRotor.omega_n    = 1.4*35.9;      % 主旋翼一阶挥舞固有频率 
UpperRotor.rho        = rho;           % 空气密度
UpperRotor.s          = 0.127;         % 主旋翼实度
UpperRotor.theta_t    = deg2rad(-10);  % 主旋翼桨叶扭转角 rad
UpperRotor.x_cg       = 0;             % 主旋翼处于重心之后的位置
UpperRotor.x_H        = 0;             % 桨毂x位置
UpperRotor.y_H        = 0;             % 桨毂y位置
UpperRotor.z_H        = -0.89-0.77;    % 桨毂z位置
UpperRotor.z_diff     = 0.77;          % 上下旋翼间距 m
UpperRotor.I_beta     = 450;           % 主旋翼挥舞惯性矩 kg m^2
UpperRotor.K_beta     = 183397;        % K_beta 弹簧刚度  Nm/rad
UpperRotor.M_beta     = 123;           % 主旋翼对挥舞铰的质量静矩
UpperRotor.Omega      = 35.9;          % 主旋翼转速rad/s
UpperRotor.R          = 5.49;          % 主旋翼半径 m

%% 实例1：悬停
DoubleRotorHelicopter.u         = 0;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

LowerRotor.u        = DoubleRotorHelicopter.u;
LowerRotor.v        = DoubleRotorHelicopter.v;
LowerRotor.w        = DoubleRotorHelicopter.w;
LowerRotor.u_dot    = DoubleRotorHelicopter.u_dot;
LowerRotor.v_dot    = DoubleRotorHelicopter.v_dot;
LowerRotor.w_dot    = DoubleRotorHelicopter.w_dot;
LowerRotor.p        = DoubleRotorHelicopter.p;
LowerRotor.q        = DoubleRotorHelicopter.q;
LowerRotor.r        = DoubleRotorHelicopter.r;
LowerRotor.p_dot    = DoubleRotorHelicopter.p_dot;
LowerRotor.q_dot    = DoubleRotorHelicopter.q_dot;
LowerRotor.r_dot    = DoubleRotorHelicopter.r_dot;

LowerRotor.v_0          = 10;
LowerRotor.theta_0      = deg2rad(15);
LowerRotor.theta_1s     = deg2rad(-2);
LowerRotor.theta_1c     = deg2rad(0);
LowerRotor.calculate_flapping_angle_3blades();
LowerRotor.calculate_force();
LowerRotor.calculate_torque();

UpperRotor.u        = DoubleRotorHelicopter.u;
UpperRotor.v        = -DoubleRotorHelicopter.v;
UpperRotor.w        = DoubleRotorHelicopter.w;
UpperRotor.u_dot    = DoubleRotorHelicopter.u_dot;
UpperRotor.v_dot    = -DoubleRotorHelicopter.v_dot;
UpperRotor.w_dot    = DoubleRotorHelicopter.w_dot;
UpperRotor.p        = -DoubleRotorHelicopter.p;
UpperRotor.q        = DoubleRotorHelicopter.q;
UpperRotor.r        = -DoubleRotorHelicopter.r;
UpperRotor.p_dot    = -DoubleRotorHelicopter.p_dot;
UpperRotor.q_dot    = DoubleRotorHelicopter.q_dot;
UpperRotor.r_dot    = -DoubleRotorHelicopter.r_dot;

UpperRotor.v_0          = 10;
UpperRotor.theta_0      = deg2rad(15);
UpperRotor.theta_1s     = deg2rad(-2);
UpperRotor.theta_1c     = deg2rad(0);
UpperRotor.calculate_flapping_angle_3blades();
UpperRotor.calculate_force();
UpperRotor.calculate_torque();

%% 实例2 前飞
disp('前飞')
DoubleRotorHelicopter.u         = 10;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

LowerRotor.u        = DoubleRotorHelicopter.u;
LowerRotor.v        = DoubleRotorHelicopter.v;
LowerRotor.w        = DoubleRotorHelicopter.w;
LowerRotor.u_dot    = DoubleRotorHelicopter.u_dot;
LowerRotor.v_dot    = DoubleRotorHelicopter.v_dot;
LowerRotor.w_dot    = DoubleRotorHelicopter.w_dot;
LowerRotor.p        = DoubleRotorHelicopter.p;
LowerRotor.q        = DoubleRotorHelicopter.q;
LowerRotor.r        = DoubleRotorHelicopter.r;
LowerRotor.p_dot    = DoubleRotorHelicopter.p_dot;
LowerRotor.q_dot    = DoubleRotorHelicopter.q_dot;
LowerRotor.r_dot    = DoubleRotorHelicopter.r_dot;

% LowerRotor.u_dot_H      = 0;
% LowerRotor.v_dot_H      = 0;
% LowerRotor.w_dot_H      = 0;

LowerRotor.v_0          = 10;
LowerRotor.theta_0      = deg2rad(15);
LowerRotor.theta_1s     = deg2rad(0);
LowerRotor.theta_1c     = deg2rad(0);
LowerRotor.calculate_flapping_angle_3blades();
LowerRotor.calculate_force();
LowerRotor.calculate_torque();

UpperRotor.u        = DoubleRotorHelicopter.u;
UpperRotor.v        = -DoubleRotorHelicopter.v;
UpperRotor.w        = DoubleRotorHelicopter.w;
UpperRotor.u_dot    = DoubleRotorHelicopter.u_dot;
UpperRotor.v_dot    = -DoubleRotorHelicopter.v_dot;
UpperRotor.w_dot    = DoubleRotorHelicopter.w_dot;
UpperRotor.p        = -DoubleRotorHelicopter.p;
UpperRotor.q        = DoubleRotorHelicopter.q;
UpperRotor.r        = -DoubleRotorHelicopter.r;
UpperRotor.p_dot    = -DoubleRotorHelicopter.p_dot;
UpperRotor.q_dot    = DoubleRotorHelicopter.q_dot;
UpperRotor.r_dot    = -DoubleRotorHelicopter.r_dot;

% UpperRotor.u_dot_H      = 0;
% UpperRotor.v_dot_H      = 0;
% UpperRotor.w_dot_H      = 0;

UpperRotor.v_0          = 10;
UpperRotor.theta_0      = deg2rad(15);
UpperRotor.theta_1s     = deg2rad(0);
UpperRotor.theta_1c     = deg2rad(0);
UpperRotor.calculate_flapping_angle_3blades();
UpperRotor.calculate_force();
UpperRotor.calculate_torque();

%% 实例2 右飞
disp('右飞')
DoubleRotorHelicopter.u         = 0;
DoubleRotorHelicopter.v         = 10;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

LowerRotor.u        = DoubleRotorHelicopter.u;
LowerRotor.v        = DoubleRotorHelicopter.v;
LowerRotor.w        = DoubleRotorHelicopter.w;
LowerRotor.u_dot    = DoubleRotorHelicopter.u_dot;
LowerRotor.v_dot    = DoubleRotorHelicopter.v_dot;
LowerRotor.w_dot    = DoubleRotorHelicopter.w_dot;
LowerRotor.p        = DoubleRotorHelicopter.p;
LowerRotor.q        = DoubleRotorHelicopter.q;
LowerRotor.r        = DoubleRotorHelicopter.r;
LowerRotor.p_dot    = DoubleRotorHelicopter.p_dot;
LowerRotor.q_dot    = DoubleRotorHelicopter.q_dot;
LowerRotor.r_dot    = DoubleRotorHelicopter.r_dot;

% LowerRotor.u_dot_H      = 0;
% LowerRotor.v_dot_H      = 0;
% LowerRotor.w_dot_H      = 0;

LowerRotor.v_0          = 10;
LowerRotor.theta_0      = deg2rad(15);
LowerRotor.theta_1s     = deg2rad(0);
LowerRotor.theta_1c     = deg2rad(0);
LowerRotor.calculate_flapping_angle_3blades();
LowerRotor.calculate_force();
LowerRotor.calculate_torque();

UpperRotor.u        = DoubleRotorHelicopter.u;
UpperRotor.v        = -DoubleRotorHelicopter.v;
UpperRotor.w        = DoubleRotorHelicopter.w;
UpperRotor.u_dot    = DoubleRotorHelicopter.u_dot;
UpperRotor.v_dot    = -DoubleRotorHelicopter.v_dot;
UpperRotor.w_dot    = DoubleRotorHelicopter.w_dot;
UpperRotor.p        = -DoubleRotorHelicopter.p;
UpperRotor.q        = DoubleRotorHelicopter.q;
UpperRotor.r        = -DoubleRotorHelicopter.r;
UpperRotor.p_dot    = -DoubleRotorHelicopter.p_dot;
UpperRotor.q_dot    = DoubleRotorHelicopter.q_dot;
UpperRotor.r_dot    = -DoubleRotorHelicopter.r_dot;

% UpperRotor.u_dot_H      = 0;
% UpperRotor.v_dot_H      = 0;
% UpperRotor.w_dot_H      = 0;

UpperRotor.v_0          = 10;
UpperRotor.theta_0      = deg2rad(15);
UpperRotor.theta_1s     = deg2rad(0);
UpperRotor.theta_1c     = deg2rad(0);
UpperRotor.calculate_flapping_angle_3blades();
UpperRotor.calculate_force();
UpperRotor.calculate_torque();

%% 实例3 左飞
disp('左飞')
DoubleRotorHelicopter.u         = 0;
DoubleRotorHelicopter.v         = -10;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

LowerRotor.u        = DoubleRotorHelicopter.u;
LowerRotor.v        = DoubleRotorHelicopter.v;
LowerRotor.w        = DoubleRotorHelicopter.w;
LowerRotor.u_dot    = DoubleRotorHelicopter.u_dot;
LowerRotor.v_dot    = DoubleRotorHelicopter.v_dot;
LowerRotor.w_dot    = DoubleRotorHelicopter.w_dot;
LowerRotor.p        = DoubleRotorHelicopter.p;
LowerRotor.q        = DoubleRotorHelicopter.q;
LowerRotor.r        = DoubleRotorHelicopter.r;
LowerRotor.p_dot    = DoubleRotorHelicopter.p_dot;
LowerRotor.q_dot    = DoubleRotorHelicopter.q_dot;
LowerRotor.r_dot    = DoubleRotorHelicopter.r_dot;

% LowerRotor.u_dot_H      = 0;
% LowerRotor.v_dot_H      = 0;
% LowerRotor.w_dot_H      = 0;

LowerRotor.v_0          = 10;
LowerRotor.theta_0      = deg2rad(15);
LowerRotor.theta_1s     = deg2rad(0);
LowerRotor.theta_1c     = deg2rad(0);
LowerRotor.calculate_flapping_angle_3blades();
LowerRotor.calculate_force();
LowerRotor.calculate_torque();

UpperRotor.u        = DoubleRotorHelicopter.u;
UpperRotor.v        = -DoubleRotorHelicopter.v;
UpperRotor.w        = DoubleRotorHelicopter.w;
UpperRotor.u_dot    = DoubleRotorHelicopter.u_dot;
UpperRotor.v_dot    = -DoubleRotorHelicopter.v_dot;
UpperRotor.w_dot    = DoubleRotorHelicopter.w_dot;
UpperRotor.p        = -DoubleRotorHelicopter.p;
UpperRotor.q        = DoubleRotorHelicopter.q;
UpperRotor.r        = -DoubleRotorHelicopter.r;
UpperRotor.p_dot    = -DoubleRotorHelicopter.p_dot;
UpperRotor.q_dot    = DoubleRotorHelicopter.q_dot;
UpperRotor.r_dot    = -DoubleRotorHelicopter.r_dot;

% UpperRotor.u_dot_H      = 0;
% UpperRotor.v_dot_H      = 0;
% UpperRotor.w_dot_H      = 0;

UpperRotor.v_0          = 10;
UpperRotor.theta_0      = deg2rad(15);
UpperRotor.theta_1s     = deg2rad(0);
UpperRotor.theta_1c     = deg2rad(0);
UpperRotor.calculate_flapping_angle_3blades();
UpperRotor.calculate_force();
UpperRotor.calculate_torque();

%% 实例4 后飞
disp('后飞')
DoubleRotorHelicopter.u         = 10;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.u_dot     = 0;
DoubleRotorHelicopter.v_dot     = 0;
DoubleRotorHelicopter.w_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

LowerRotor.u        = DoubleRotorHelicopter.u;
LowerRotor.v        = DoubleRotorHelicopter.v;
LowerRotor.w        = DoubleRotorHelicopter.w;
LowerRotor.u_dot    = DoubleRotorHelicopter.u_dot;
LowerRotor.v_dot    = DoubleRotorHelicopter.v_dot;
LowerRotor.w_dot    = DoubleRotorHelicopter.w_dot;
LowerRotor.p        = DoubleRotorHelicopter.p;
LowerRotor.q        = DoubleRotorHelicopter.q;
LowerRotor.r        = DoubleRotorHelicopter.r;
LowerRotor.p_dot    = DoubleRotorHelicopter.p_dot;
LowerRotor.q_dot    = DoubleRotorHelicopter.q_dot;
LowerRotor.r_dot    = DoubleRotorHelicopter.r_dot;

% LowerRotor.u_dot_H      = 0;
% LowerRotor.v_dot_H      = 0;
% LowerRotor.w_dot_H      = 0;

LowerRotor.v_0          = 10;
LowerRotor.theta_0      = deg2rad(15);
LowerRotor.theta_1s     = deg2rad(0);
LowerRotor.theta_1c     = deg2rad(0);
LowerRotor.calculate_flapping_angle_3blades();
LowerRotor.calculate_force();
LowerRotor.calculate_torque();

UpperRotor.u        = DoubleRotorHelicopter.u;
UpperRotor.v        = -DoubleRotorHelicopter.v;     %镜像
UpperRotor.w        = DoubleRotorHelicopter.w;
UpperRotor.u_dot    = DoubleRotorHelicopter.u_dot;
UpperRotor.v_dot    = -DoubleRotorHelicopter.v_dot; %镜像
UpperRotor.w_dot    = DoubleRotorHelicopter.w_dot;
UpperRotor.p        = -DoubleRotorHelicopter.p;     %镜像
UpperRotor.q        = DoubleRotorHelicopter.q;
UpperRotor.r        = -DoubleRotorHelicopter.r;     %镜像
UpperRotor.p_dot    = -DoubleRotorHelicopter.p_dot; %镜像
UpperRotor.q_dot    = DoubleRotorHelicopter.q_dot;
UpperRotor.r_dot    = -DoubleRotorHelicopter.r_dot; %镜像

% UpperRotor.u_dot_H      = 0;
% UpperRotor.v_dot_H      = 0;
% UpperRotor.w_dot_H      = 0;

UpperRotor.v_0          = 10;
UpperRotor.theta_0      = deg2rad(15);
UpperRotor.theta_1s     = deg2rad(0);
UpperRotor.theta_1c     = deg2rad(0);
UpperRotor.calculate_flapping_angle_3blades();
UpperRotor.calculate_force();
UpperRotor.calculate_torque();

