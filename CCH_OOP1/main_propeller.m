% 2.12
clear all
clc
h = 10;
[~,~,~,rho] = atmosisa(h);

SingleRotorHelicopter = Helicopter();

%% private 属性设置
Prop = Propeller();
Prop.a_0        = 5.7;
Prop.b          = 4;
Prop.c          = 0.2042;
Prop.delta      = 0.008;
Prop.h_R        = 0;
Prop.rho        = rho;
Prop.s          = 0.2;
Prop.theta_t    = deg2rad(-30);
Prop.x_cg       = 7.66;
Prop.x_H        = -7.66;
Prop.y_H        = 0;
Prop.z_H        = 0;
Prop.Omega      = 162;
Prop.R          = 1.3;

%% 实例1：悬停
SingleRotorHelicopter.u         = 13;
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

Prop.u        = SingleRotorHelicopter.u;
Prop.v        = SingleRotorHelicopter.v;
Prop.w        = SingleRotorHelicopter.w;
Prop.u_dot    = SingleRotorHelicopter.u_dot;
Prop.v_dot    = SingleRotorHelicopter.v_dot;
Prop.w_dot    = SingleRotorHelicopter.w_dot;
Prop.p        = SingleRotorHelicopter.p;
Prop.q        = SingleRotorHelicopter.q;
Prop.r        = SingleRotorHelicopter.r;
Prop.p_dot    = SingleRotorHelicopter.p_dot;
Prop.q_dot    = SingleRotorHelicopter.q_dot;
Prop.r_dot    = SingleRotorHelicopter.r_dot;

Prop.v_0          = 10;
Prop.theta_0      = deg2rad(15);

Prop.calculate_force();

%%

