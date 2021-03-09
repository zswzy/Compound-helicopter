function Rotorcraft = copy_states(Rotorcraft)
%COPY_STATES 
%   Rotorcraft: Rotorcraft 结构体
%   复制状态变量
DoubleRotorHelicopter   = Rotorcraft.DoubleRotorHelicopter;
LowerRotor              = Rotorcraft.LowerRotor;
UpperRotor              = Rotorcraft.UpperRotor;
Prop                    = Rotorcraft.Prop;
Fus                     = Rotorcraft.Fus;
HorStab                 = Rotorcraft.HorStab;
VerStab                 = Rotorcraft.VerStab;

LowerRotor.U        = DoubleRotorHelicopter.U;
LowerRotor.V        = LowerRotor.direction * DoubleRotorHelicopter.V;
LowerRotor.W        = DoubleRotorHelicopter.W;
LowerRotor.u        = DoubleRotorHelicopter.u;
LowerRotor.v        = LowerRotor.direction * DoubleRotorHelicopter.v;
LowerRotor.w        = DoubleRotorHelicopter.w;
LowerRotor.u_dot    = DoubleRotorHelicopter.u_dot;
LowerRotor.v_dot    = LowerRotor.direction * DoubleRotorHelicopter.v_dot;
LowerRotor.w_dot    = DoubleRotorHelicopter.w_dot;
LowerRotor.p        = LowerRotor.direction * DoubleRotorHelicopter.p;
LowerRotor.q        = DoubleRotorHelicopter.q;
LowerRotor.r        = LowerRotor.direction * DoubleRotorHelicopter.r;
LowerRotor.p_dot    = LowerRotor.direction * DoubleRotorHelicopter.p_dot;
LowerRotor.q_dot    = DoubleRotorHelicopter.q_dot;
LowerRotor.r_dot    = LowerRotor.direction * DoubleRotorHelicopter.r_dot;

UpperRotor.U        = DoubleRotorHelicopter.U;
UpperRotor.V        = UpperRotor.direction * DoubleRotorHelicopter.V;
UpperRotor.W        = DoubleRotorHelicopter.W;
UpperRotor.u        = DoubleRotorHelicopter.u;
UpperRotor.v        = UpperRotor.direction * DoubleRotorHelicopter.v;
UpperRotor.w        = DoubleRotorHelicopter.w;
UpperRotor.u_dot    = DoubleRotorHelicopter.u_dot;
UpperRotor.v_dot    = UpperRotor.direction * DoubleRotorHelicopter.v_dot;
UpperRotor.w_dot    = DoubleRotorHelicopter.w_dot;
UpperRotor.p        = UpperRotor.direction * DoubleRotorHelicopter.p;
UpperRotor.q        = DoubleRotorHelicopter.q;
UpperRotor.r        = UpperRotor.direction * DoubleRotorHelicopter.r;
UpperRotor.p_dot    = UpperRotor.direction * DoubleRotorHelicopter.p_dot;
UpperRotor.q_dot    = DoubleRotorHelicopter.q_dot;
UpperRotor.r_dot    = UpperRotor.direction * DoubleRotorHelicopter.r_dot;

Prop.U               = DoubleRotorHelicopter.U;
Prop.V               = DoubleRotorHelicopter.V;
Prop.W               = DoubleRotorHelicopter.W;
Prop.u               = DoubleRotorHelicopter.u;
Prop.v               = DoubleRotorHelicopter.v;
Prop.w               = DoubleRotorHelicopter.w;
Prop.p               = DoubleRotorHelicopter.p;
Prop.q               = DoubleRotorHelicopter.q;
Prop.r               = DoubleRotorHelicopter.r;

Fus.U               = DoubleRotorHelicopter.U;
Fus.V               = DoubleRotorHelicopter.V;
Fus.W               = DoubleRotorHelicopter.W;
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

% 返回结构体
Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
Rotorcraft.LowerRotor               = LowerRotor;
Rotorcraft.UpperRotor               = UpperRotor;
Rotorcraft.Prop                     = Prop;
Rotorcraft.Fus                      = Fus;
Rotorcraft.HorStab                  = HorStab;  
Rotorcraft.VerStab                  = VerStab;
end

