% 建立对象

% 直升机
DoubleRotorHelicopter       = Helicopter();
DoubleRotorHelicopter.GW    = 5500;
DoubleRotorHelicopter.GWF   = 5500*9.81;
DoubleRotorHelicopter.Ix    = 5518;
DoubleRotorHelicopter.Iy    = 26844;
DoubleRotorHelicopter.Iz    = 23048;
DoubleRotorHelicopter.Ixz   = 0;

% 下旋翼
LowerRotor = RotorFixed('anticlockwise'); % 下旋翼逆时针转， 为默认情况
LowerRotor.a_0          = 7.6;           % 主旋翼升力线斜率,NACA0012
LowerRotor.b            = 3;             % 旋翼桨叶数
LowerRotor.c            = 0.29;          % 主旋翼桨叶弦长m
LowerRotor.delta        = 0.01;         % 主旋翼桨叶阻力系数
LowerRotor.e            = 2.58/5.49;     % 无量纲等效铰偏置量
LowerRotor.e_oswald     = 0.7;           % 奥斯瓦尔德效率因子 (0.8)
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
LowerRotor.K_beta       = 220500;        % K_beta 弹簧刚度  Nm/rad
LowerRotor.M_beta       = 123;           % 主旋翼对挥舞铰的质量静矩
LowerRotor.Omega        = 35.9;          % 主旋翼转速rad/s
LowerRotor.R            = 5.49;          % 主旋翼半径 m

% 上旋翼
UpperRotor = RotorFixed('clockwise'); % 上旋翼顺时针转
UpperRotor.a_0          = 7.6;           % 主旋翼升力线斜率,NACA0012
UpperRotor.b            = 3;             % 旋翼桨叶数
UpperRotor.c            = 0.29;          % 主旋翼桨叶弦长m
UpperRotor.delta        = 0.01;         % 主旋翼桨叶阻力系数
UpperRotor.e            = 2.58/5.49;     % 无量纲等效铰偏置量
UpperRotor.e_oswald     = 0.7;           % 奥斯瓦尔德效率因子 (0.8)
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
UpperRotor.K_beta       = 220500;        % K_beta 弹簧刚度  Nm/rad
UpperRotor.M_beta       = 123;           % 主旋翼对挥舞铰的质量静矩
UpperRotor.Omega        = 35.9;          % 主旋翼转速rad/s
UpperRotor.R            = 5.49;          % 主旋翼半径 m

% 推进桨
% Prop = PropellerNUAA();
% Prop.delta              = 0.008;
% Prop.e_oswald           = 1;
% Prop.rho                = rho;
% Prop.s                  = 0.2;
% Prop.x_PR                = -7.66;
% Prop.y_PR                = 0;
% Prop.z_PR                = 0;
% Prop.Omega              = 162;
% Prop.R                  = 1.3;

Prop = PropellerKevin();
Prop.c = 0.2;
Prop.e_oswald           = 0.8;
Prop.Nb                 = 4;
Prop.Omega              = 162;
Prop.R                  = 1.3;
Prop.s                  = 0.2;
Prop.theta_t            = deg2rad(-30);
Prop.rho                = 1.2;
Prop.x_PR               = -7.66;
Prop.y_PR               = 0;
Prop.z_PR               = 0;

% 机身
Fus = Fuselage();
Fus.rho                 = rho;
Fus.R                   = 5.49;
Fus.A                   = pi*5.49^2;
Fus.s_F                 = (5.6+5.6)*0.1+pi*1.5^2;
Fus.x_F                 = 0;
Fus.y_F                 = 0;
Fus.z_F                 = 0;

% % 平尾
% HorStab = HorizontalStabilizerSimple();
% HorStab.a_0             = 3.2;
% HorStab.delta           = 0.001;
% HorStab.rho             = rho;
% HorStab.s_HS            = 5.6;
% HorStab.s_e             = 5.6*0.1;
% HorStab.xi              = 0;
% HorStab.K_HS            = 0.8;
% HorStab.x_HS            = -6.8;
% HorStab.y_HS            = 0;
% HorStab.z_HS            = 0.2;
% 
% % 垂尾
% VerStab = VerticalStabilizerSimple();
% VerStab.a_0             = 2;
% VerStab.delta           = 0.001;
% VerStab.rho             = rho;
% VerStab.s_VS            = 5.6;
% VerStab.s_r             = 5.6*0.1;
% VerStab.xi              = 0;
% VerStab.K_VS            = 0.8;
% VerStab.x_VS            = -6.8;
% VerStab.y_VS            = 0;
% VerStab.z_VS            = -0.5;

% Horizontal Stabilizer
HorStab = HorizontalStabilizerNASA();
HorStab.rho             = rho;
HorStab.R               = 5.49;
HorStab.Omega           = 35.9;
HorStab.x_HS            = -6.8;
HorStab.y_HS            = 0;
HorStab.z_HS            = 0.2;

% Vertical Stabilizer
VerStab = VerticalStabilizerNASA();
VerStab.rho             = rho;
VerStab.R               = 5.49;
VerStab.Omega           = 35.9;
VerStab.x_VS            = -6.8;
VerStab.y_VS            = 0;
VerStab.z_VS            = -0.5;

% 建立结构体
Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
Rotorcraft.LowerRotor               = LowerRotor;
Rotorcraft.UpperRotor               = UpperRotor;
Rotorcraft.Prop                     = Prop;
Rotorcraft.Fus                      = Fus;
Rotorcraft.HorStab                  = HorStab;  
Rotorcraft.VerStab                  = VerStab;