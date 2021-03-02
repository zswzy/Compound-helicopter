%% Main file
clear all
clc

%% 定义基本参数
global Heli% 直升机结构体，内含运动状态
global Control % 操纵变量结构体，内含操纵变量
% 飞行高度(海平面）
global h
h = 10;

% 大气数据
global rho
[~,~,~,rho] = atmosisa(h); %温度，音速，压强，密度
% 重力加速度
global g
g = 9.81;

global Rotor b m_b z_diff R A Omega s theta_t omega_n I_beta a_0 delta c K_beta gamma_M  e M_beta gamma_s x_H1 y_H1 z_H1 x_H2 y_H2 z_H2 h_R1 h_R2 x_cg
% 上下旋翼各自桨叶数
b = 3;
% 桨叶质量
m_b = 60;
% 上下旋翼间距 m
z_diff = 0.77;
% 主旋翼半径 m
R = 5.49;
%桨盘面积
A = pi*R^2;
% 主旋翼转速rad/s
Omega = 35.9;
% 主旋翼实度
s = 0.127;
% 主旋翼桨叶扭转角 rad
theta_t = deg2rad(-10);
% 主旋翼一阶挥舞固有频率 
omega_n = 1.4*Omega;
% 主旋翼挥舞惯性矩 kg m^2
I_beta = 450;
% 主旋翼升力线斜率,NACA0012
a_0 = 5.7;
%主旋翼桨叶阻力系数
delta = 0.008;
% 主旋翼桨叶弦长m
c = 0.29;
% K_beta 弹簧刚度  Nm/rad
% K_beta = 220500;
K_beta = 183397;
% 主旋翼洛克数 
gamma_M = 5.41;
% 无量纲等效铰偏置量epsilon 0.4左右,e为带量纲的
e = 2*((omega_n/Omega)^2-1)/(1+2*(omega_n/Omega)^2);
% e = 2.58/R;
% 主旋翼对挥舞铰的质量静矩 200多
% M_beta = 1/2*R^2*m_b/R; 
% M_beta = I_beta*3/2*(1/R)/(1-epsilon);
M_beta = 123;
% 主旋翼桨毂纵向安装角
gamma_s = deg2rad(0);
%gamma_s = deg2rad(0);
% 主旋翼桨毂横向安装角
i_phi = 0;
% 主旋翼位置 1：下旋翼，2：上旋翼
h_R1 = 0.89;
h_R2 = 0.89+z_diff;
x_cg = 0;
x_H1 = 0;
y_H1 = 0;
z_H1 = -0.89;
x_H2 = 0;
y_H2 = 0;
z_H2 = z_H1-z_diff;


global Propeller R_PR A_PR Omega_PR s_PR theta_tPR  a_PR delta_PR x_PR y_PR z_PR K_PR I_beta_PR c_PR kappa
% 推进桨半径 m
R_PR = 1.3;
% 推进桨桨盘面积
A_PR = R_PR^2*pi;
% 推进桨转速 rad/s 162
Omega_PR = 162;
% 推进桨实度
s_PR = 0.2;
% 推进桨叶扭转
theta_tPR = deg2rad(-30);
% 推进桨叶升力线斜率,NACA0012
a_PR = 5.7;
% 推进桨叶阻力系数
delta_PR = 0.008;
% 推进桨叶坐标
x_PR = -7.66;
y_PR = 0;
z_PR = 0;
% 动压损失系数
K_PR=0.85;
% 挥舞惯性矩 kg/m^2
I_beta_PR = 21;
% 推进桨桨叶弦长
c_PR=0.14;
% 叶尖损失系数
kappa = 1;

global GW Ixx Iyy Izz 
% 整机质量 kg
GW = 5500;
%转动惯量
Ixx = 5518;
Iyy = 26844;
Izz = 23048;

global x_F y_F z_F l_F s_F Fuselage
% 机身气动中心
x_F = 0;
y_F = 0;
z_F = 0;
% 机身长度
l_F = 12;
% 机身最大迎风面积
s_F = 2.4*10;

global alpha_0HS s_H s_e xi_H x_HS y_HS z_HS Tail
% 平尾安装角
alpha_0HS = 0;
% 平尾面积
s_H = 5.6;
% 升降舵面积
s_e = 0.3*s_H;
% 平尾后掠角
xi_H = 0;
% 平尾位置
x_HS = -6.8;
y_HS = 0;
z_HS = 0.2;

global alpha_0VS x_VS y_VS z_VS s_r xi_V s_V Fin
% 垂尾安装角
alpha_0VS = 0;
% 垂尾位置
x_VS = -6.8;
y_VS = 0;
z_VS = -0.5;
% 垂尾后掠角
xi_V = 0;
% 垂尾面积
s_V = 2.79;
% 垂尾舵面面积
s_r = 0.3*s_V;

global L_BH
% 转换矩阵
L_BH = [cos(gamma_s)    0     -sin(gamma_s);
        0               1     0;
        sin(gamma_s)    0     cos(gamma_s)];
    
%% 1.24 LowerRotor
global theta_0
theta_0 = deg2rad(12);

InitialStates = 10; %v_0

options=optimset('Display','off','TolFun',1e-10,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,fval,exitflag,~] = fsolve(@LowerRotor,InitialStates,options);
disp(Rotor)

%%
C_Zh_list = [];
C_Q0_list = [];
for theta_0 = linspace(deg2rad(10),deg2rad(20),50)
    InitialStates = 10;
    [x,fval,exitflag,~] = fsolve(@LowerRotor,InitialStates,options);
    C_Zh_list = [C_Zh_list Rotor.C_Zh];
    C_Q0_list = [C_Q0_list Rotor.C_Q0];
end
plot(C_Q0_list,-C_Zh_list)

    
%% 1.24 LowerRotorS
global theta_0
theta_0 = deg2rad(12);

InitialStates = 10; %v_0

options=optimset('Display','off','TolFun',1e-10,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
[x,fval,exitflag,~] = fsolve(@LowerRotorS,InitialStates,options);
disp(Rotor)
