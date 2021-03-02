function copy_states()
%COPY_STATES 此处显示有关此函数的摘要
%   此处显示详细说明
global DoubleRotorHelicopter
global LowerRotor UpperRotor
global Prop
global Fus
global HorStab
global VerStab

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
end

