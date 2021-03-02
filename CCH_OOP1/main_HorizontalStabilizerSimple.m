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

% 平尾
HorStab = HorizontalStabilizerSimple();

HorStab.a_0     = 3.5;
HorStab.delta   = 0.008;
HorStab.rho     = rho;
HorStab.s_HS    = 5.6;
HorStab.s_e     = 5.6*0.1;
HorStab.xi      = 0;
HorStab.x_HS    = -6.8;
HorStab.y_HS    = 0;
HorStab.z_HS    = 0.2;

DoubleRotorHelicopter.u         = 0;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = 0;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;

HorStab.u = DoubleRotorHelicopter.u;
HorStab.v = DoubleRotorHelicopter.v;
HorStab.w = DoubleRotorHelicopter.w;
HorStab.p = DoubleRotorHelicopter.p;
HorStab.q = DoubleRotorHelicopter.q;
HorStab.r = DoubleRotorHelicopter.r;

HorStab.delta_e = 0;
HorStab.calculate_force();
HorStab.calculate_torque();
%% instance1 前飞
DoubleRotorHelicopter.u         = 100;
DoubleRotorHelicopter.v         = 0;
DoubleRotorHelicopter.w         = 2;
DoubleRotorHelicopter.p         = 0;
DoubleRotorHelicopter.q         = 0;
DoubleRotorHelicopter.r         = 0;

HorStab.u = DoubleRotorHelicopter.u;
HorStab.v = DoubleRotorHelicopter.v;
HorStab.w = DoubleRotorHelicopter.w;
HorStab.p = DoubleRotorHelicopter.p;
HorStab.q = DoubleRotorHelicopter.q;
HorStab.r = DoubleRotorHelicopter.r;

HorStab.delta_e = deg2rad(-2);
HorStab.calculate_force();
HorStab.calculate_torque();