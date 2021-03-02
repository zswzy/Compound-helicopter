function power_total = trim_power_prop(Prop_theta_0)
%TRIM_POWER_PROP 
%   输入Prop_theta_0，输出整机功率

global DoubleRotorHelicopter
global LowerRotor UpperRotor 
global Prop
global Fus
global HorStab
global VerStab

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

Prop.u               = DoubleRotorHelicopter.u;
Prop.v               = DoubleRotorHelicopter.v;
Prop.w               = DoubleRotorHelicopter.w;
Prop.p               = DoubleRotorHelicopter.p;
Prop.q               = DoubleRotorHelicopter.q;
Prop.r               = DoubleRotorHelicopter.r;

Fus.u               = DoubleRotorHelicopter.u;
Fus.v               = DoubleRotorHelicopter.v;
Fus.w               = DoubleRotorHelicopter.w;
Fus.p               = DoubleRotorHelicopter.p;
Fus.q               = DoubleRotorHelicopter.q;
Fus.r               = DoubleRotorHelicopter.r;

HorStab.u           = DoubleRotorHelicopter.u;
HorStab.v           = DoubleRotorHelicopter.v;
HorStab.w           = DoubleRotorHelicopter.w;
HorStab.p           = DoubleRotorHelicopter.p;
HorStab.q           = DoubleRotorHelicopter.q;
HorStab.r           = DoubleRotorHelicopter.r;

VerStab.u           = DoubleRotorHelicopter.u;
VerStab.v           = DoubleRotorHelicopter.v;
VerStab.w           = DoubleRotorHelicopter.w;
VerStab.p           = DoubleRotorHelicopter.p;
VerStab.q           = DoubleRotorHelicopter.q;
VerStab.r           = DoubleRotorHelicopter.r;

% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options         = optimset('Display','iter','TolFun',1e-15,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
InitialStates   = [0.01,0,0,0,0,0,10,10];
[x,~,exitflag,~] = trim_solve(@Aerodynamics_trim_full_8var, ... 
                                InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                Prop_theta_0, ...         % Prop.theta_0
                                1, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                deg2rad(0), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                deg2rad(0), ...          % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                deg2rad(0), ...         % theta_1c_diff
                                deg2rad(0));            % theta_1s_diff% theta_1s_diff
power_total = LowerRotor.Power_total ...
                + UpperRotor.Power_total ...
                + Prop.Power_total;
display(power_total)
end

