% 2.18 整机配平
clear all
clc
h = 100;
[~,~,~,rho] = atmosisa(h);

%% 建立对象
global DoubleRotorHelicopter
global LowerRotor UpperRotor 
global Prop
global Fus
global HorStab
global VerStab

% 直升机
DoubleRotorHelicopter       = Helicopter();
DoubleRotorHelicopter.GW    = 5500;
DoubleRotorHelicopter.GWF   = 5500*9.81;

% 下旋翼
LowerRotor = RotorFixed('anticlockwise'); % 下旋翼逆时针转， 为默认情况
LowerRotor.a_0          = 5.7;           % 主旋翼升力线斜率,NACA0012
LowerRotor.b            = 3;             % 旋翼桨叶数
LowerRotor.c            = 0.29;          % 主旋翼桨叶弦长m
LowerRotor.delta        = 0.01;         % 主旋翼桨叶阻力系数
LowerRotor.e            = 2.58/5.49;     % 无量纲等效铰偏置量
LowerRotor.e_oswald     = 0.8;           % 奥斯瓦尔德效率因子 (0.8)
LowerRotor.gamma_s      = deg2rad(3);    % 主旋翼桨毂纵向安装角
LowerRotor.h_R          = 0.89;          % 主旋翼处于重心之上的位置
LowerRotor.m_b          = 60;            % 桨叶质量
LowerRotor.omega_n      = 1.4*35.9;      % 主旋翼一阶挥舞固有频率 
LowerRotor.rho          = rho;           % 空气密度
LowerRotor.s            = 0.127;         % 主旋翼实度
LowerRotor.theta_t      = deg2rad(-10);  % 主旋翼桨叶扭转角 rad
LowerRotor.x_cg         = 0;             % 主旋翼处于重心之后的位置
LowerRotor.x_H          = 0;             % 桨毂x位置
LowerRotor.y_H          = 0;             % 桨毂y位置
LowerRotor.z_H          = -0.89;         % 桨毂z位置
LowerRotor.z_diff       = 0.77;          % 上下旋翼间距 m
LowerRotor.I_beta       = 450;           % 主旋翼挥舞惯性矩 kg m^2
LowerRotor.K_beta       = 183397;        % K_beta 弹簧刚度  Nm/rad
LowerRotor.M_beta       = 123;           % 主旋翼对挥舞铰的质量静矩
LowerRotor.Omega        = 35.9;          % 主旋翼转速rad/s
LowerRotor.R            = 5.49;          % 主旋翼半径 m

% 上旋翼
UpperRotor = RotorFixed('clockwise'); % 上旋翼顺时针转
UpperRotor.a_0          = 5.7;           % 主旋翼升力线斜率,NACA0012
UpperRotor.b            = 3;             % 旋翼桨叶数
UpperRotor.c            = 0.29;          % 主旋翼桨叶弦长m
UpperRotor.delta        = 0.01;         % 主旋翼桨叶阻力系数
UpperRotor.e            = 2.58/5.49;     % 无量纲等效铰偏置量
UpperRotor.e_oswald     = 0.8;           % 奥斯瓦尔德效率因子 (0.8)
UpperRotor.gamma_s      = deg2rad(3);    % 主旋翼桨毂纵向安装角
UpperRotor.h_R          = 0.89+0.77;     % 主旋翼处于重心之上的位置
UpperRotor.m_b          = 60;            % 桨叶质量
UpperRotor.omega_n      = 1.4*35.9;      % 主旋翼一阶挥舞固有频率 
UpperRotor.rho          = rho;           % 空气密度
UpperRotor.s            = 0.127;         % 主旋翼实度
UpperRotor.theta_t      = deg2rad(-10);  % 主旋翼桨叶扭转角 rad
UpperRotor.x_cg         = 0;             % 主旋翼处于重心之后的位置
UpperRotor.x_H          = 0;             % 桨毂x位置
UpperRotor.y_H          = 0;             % 桨毂y位置
UpperRotor.z_H          = -0.89-0.77;    % 桨毂z位置
UpperRotor.z_diff       = 0.77;          % 上下旋翼间距 m
UpperRotor.I_beta       = 450;           % 主旋翼挥舞惯性矩 kg m^2
UpperRotor.K_beta       = 183397;        % K_beta 弹簧刚度  Nm/rad
UpperRotor.M_beta       = 123;           % 主旋翼对挥舞铰的质量静矩
UpperRotor.Omega        = 35.9;          % 主旋翼转速rad/s
UpperRotor.R            = 5.49;          % 主旋翼半径 m

% 推进桨
Prop = PropellerNUAA();
Prop.delta              = 0.008;
Prop.e_oswald           = 0.8;
Prop.rho                = rho;
Prop.s                  = 0.2;
Prop.x_PR                = -7.66;
Prop.y_PR                = 0;
Prop.z_PR                = 0;
Prop.Omega              = 162;
Prop.R                  = 1.3;

% 机身
Fus = Fuselage();
Fus.rho                 = rho;
Fus.R                   = 5.49;
Fus.A                   = pi*5.49^2;
Fus.s_F                 = (5.6+5.6)*0.1+pi*1.5^2;
Fus.x_F                 = 0;
Fus.y_F                 = 0;
Fus.z_F                 = 0;

% 平尾
HorStab = HorizontalStabilizerSimple();
HorStab.a_0             = 3.5;
HorStab.delta           = 0.008;
HorStab.rho             = rho;
HorStab.s_HS            = 5.6;
HorStab.s_e             = 5.6*0.1;
HorStab.xi              = 0;
HorStab.x_HS            = -6.8;
HorStab.y_HS            = 0;
HorStab.z_HS            = 0.2;

% 垂尾
VerStab = VerticalStabilizerSimple();
VerStab.a_0             = 2.79;
VerStab.delta           = 0.008;
VerStab.rho             = rho;
VerStab.s_VS            = 5.6;
VerStab.s_r             = 5.6*0.1;
VerStab.xi              = 0;
VerStab.x_VS            = -6.8;
VerStab.y_VS            = 0;
VerStab.z_VS            = -0.5;

%% 悬停/前飞配平 X,Y,Z,L,M,N=0,[theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
DoubleRotorHelicopter.U         = 42;
DoubleRotorHelicopter.V         = 0;
DoubleRotorHelicopter.W         = 0;
DoubleRotorHelicopter.U_dot     = 0;
DoubleRotorHelicopter.V_dot     = 0;
DoubleRotorHelicopter.W_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options         = optimset('Display','iter','TolFun',1e-15,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
InitialStates   = [0.01,0,0,0,0,0,10,10];
[x,~,exitflag,~,power_total] = trim_solve(@Aerodynamics_trim_full_8var, ... 
                                InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                deg2rad(15), ...         % Prop.theta_0
                                0, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                deg2rad(0), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                deg2rad(0), ...          % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                deg2rad(0), ...         % theta_1c_diff
                                deg2rad(0));            % theta_1s_diff
%%
DoubleRotorHelicopter.U         = 100;
DoubleRotorHelicopter.V         = 0;
DoubleRotorHelicopter.W         = 0;
DoubleRotorHelicopter.U_dot     = 0;
DoubleRotorHelicopter.V_dot     = 0;
DoubleRotorHelicopter.W_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;
[~,~,~,~] = power_compare_prop(@Aerodynamics_trim_full_8var, ... 
                    2, ...                  % LowerRotor.inteference
                    2, ...                  % UpperRotor.inteference
                    deg2rad(15), ...         % Prop.theta_0
                    1, ...                  % Fus.isEnable
                    deg2rad(0), ...         % HorStab.delta_e
                    1, ...                  % HorStab.isEnable
                    deg2rad(0), ...          % VerStab.delta_r
                    1, ...                  % VerStab.isEnable
                    deg2rad(0), ...         % theta_1c_diff
                    deg2rad(0));            % theta_1s_diff
%% 寻找在某一配平速度下的最优尾推
DoubleRotorHelicopter.U         = 150; 
DoubleRotorHelicopter.V         = 0;
DoubleRotorHelicopter.W         = 0;
DoubleRotorHelicopter.U_dot     = 0;
DoubleRotorHelicopter.V_dot     = 0;
DoubleRotorHelicopter.W_dot     = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;
DoubleRotorHelicopter.p_dot     = 0;
DoubleRotorHelicopter.q_dot     = 0;
DoubleRotorHelicopter.r_dot     = 0;

% 为了避免陷入局部最优，选取多个初始点
array_prop_theta_0_initial  = linspace(deg2rad(1),deg2rad(35),5); %初始点列表
array_prop_theta_0_best     = zeros(size(array_prop_theta_0_initial));
array_power_best_with_prop  = zeros(size(array_prop_theta_0_initial));

% 寻找各局部最优解
tic;
for j = 1:5
    [array_prop_theta_0_best(j),array_power_best_with_prop(j)] = fmincon(@trim_power_prop,array_prop_theta_0_initial(j),[],[],[],[],0,deg2rad(36),[],optimoptions('fmincon','Display','iter','PlotFcns',[]));
end
toc
% 寻找全局最优解
power_best_with_prop = min(array_power_best_with_prop);
prop_theta_0_best = array_prop_theta_0_best(array_power_best_with_prop == power_best_with_prop);

% 对比无尾推功率
power_best_without_prop = trim_power_prop(0,0);
if power_best_with_prop < power_best_without_prop
    prop_theta_0_optim = prop_theta_0_best;
    power_optim = power_best_with_prop;
else
    prop_theta_0_optim = nan;
    power_optim = power_best_without_prop;
end