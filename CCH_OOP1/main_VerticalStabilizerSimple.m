% 2.16 horizontal stabilizer simple测试
clearvars
clear global
clc
h = 10;
[~,~,~,rho] = atmosisa(h);

% 直升机
DoubleRotorHelicopter       = Helicopter();
DoubleRotorHelicopter.GW    = 5500;
DoubleRotorHelicopter.GWF   = 5500*9.81;

% 垂尾
VerStab = VerticalStabilizerSimple();

VerStab.a_0     = 2.79;
VerStab.delta   = 0.008;
VerStab.rho     = rho;
VerStab.s_VS    = 5.6;
VerStab.s_r     = 5.6*0.1;
VerStab.xi      = 0;
VerStab.x_VS    = -6.8;
VerStab.y_VS    = 0;
VerStab.z_VS    = -0.5;

DoubleRotorHelicopter.u         = 0;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;

VerStab.u = DoubleRotorHelicopter.u;
VerStab.v = DoubleRotorHelicopter.v;
VerStab.w = DoubleRotorHelicopter.w;
VerStab.p = DoubleRotorHelicopter.p;
VerStab.q = DoubleRotorHelicopter.q;
VerStab.r = DoubleRotorHelicopter.r;

VerStab.delta_r = 0;
VerStab.calculate_force();
VerStab.calculate_torque();
%% instance1 右前飞
DoubleRotorHelicopter.u         = 100;
DoubleRotorHelicopter.v         = 10;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;

VerStab.u = DoubleRotorHelicopter.u;
VerStab.v = DoubleRotorHelicopter.v;
VerStab.w = DoubleRotorHelicopter.w;
VerStab.p = DoubleRotorHelicopter.p;
VerStab.q = DoubleRotorHelicopter.q;
VerStab.r = DoubleRotorHelicopter.r;

VerStab.delta_r = deg2rad(-5);
VerStab.calculate_force();
VerStab.calculate_torque();