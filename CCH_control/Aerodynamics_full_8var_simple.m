function [Rotorcraft,Fnet] = Aerodynamics_full_8var_simple(Rotorcraft)
%AERODYNAMICS_FULL_8VAR 
%   用于当前飞行状态计算 
%   x: 配平变量
%   Rotorcraft: Rotorcraft 结构体
%   X,Y,Z,L,M,N=0,
%   [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
%   Fnet(1):X
%   Fnet(2):Y
%   Fnet(3):Z
%   Fnet(4):L
%   Fnet(5):M
%   Fnet(6):N

% 计算当地速度
Rotorcraft.DoubleRotorHelicopter.calculate_local_variables();
Rotorcraft = copy_states(Rotorcraft);

DoubleRotorHelicopter   = Rotorcraft.DoubleRotorHelicopter;
LowerRotor              = Rotorcraft.LowerRotor;
UpperRotor              = Rotorcraft.UpperRotor;
Prop                    = Rotorcraft.Prop;
Fus                     = Rotorcraft.Fus;
HorStab                 = Rotorcraft.HorStab;
VerStab                 = Rotorcraft.VerStab;

% 计算旋翼挥舞，旋翼力，力矩
LowerRotor.calculate_flapping_angle_3blades();
LowerRotor.calculate_force();
LowerRotor.calculate_torque();
UpperRotor.calculate_flapping_angle_3blades();
UpperRotor.calculate_force();
UpperRotor.calculate_torque();

% 计算尾推力、力矩
Prop.calculate_force();
Prop.calculate_torque();

% 计算机身力、力矩
Fus.calculate_force();
Fus.calculate_torque();

% 计算平尾力、力矩
HorStab.calculate_force();
HorStab.calculate_torque();

% 计算垂尾力、力矩
VerStab.calculate_force();
VerStab.calculate_torque();

% 计算重力
DoubleRotorHelicopter.calculate_force();
DoubleRotorHelicopter.calculate_torque();

% 构建方程 X,Y,Z,L,M,N
Fnet = ones(1,6);
Fnet(1) = LowerRotor.X ...
            + UpperRotor.X ...
            + Prop.X * Prop.isEnable ...
            + Fus.X * Fus.isEnable ...
            + HorStab.X * HorStab.isEnable ...
            + VerStab.X * VerStab.isEnable ...
            + DoubleRotorHelicopter.X;
Fnet(2) = LowerRotor.Y ...
            + UpperRotor.Y ...
            + Prop.Y * Prop.isEnable ...
            + Fus.Y * Fus.isEnable ...
            + HorStab.Y * HorStab.isEnable ...
            + VerStab.Y * VerStab.isEnable ...
            + DoubleRotorHelicopter.Y;
Fnet(3) = LowerRotor.Z ...
            + UpperRotor.Z ...
            + Prop.Z * Prop.isEnable ...
            + Fus.Z * Fus.isEnable ...
            + HorStab.Z * HorStab.isEnable ...
            + VerStab.Z * VerStab.isEnable ...
            + DoubleRotorHelicopter.Z;
Fnet(4) = LowerRotor.L ...
            + UpperRotor.L ...
            + Prop.L * Prop.isEnable ...
            + Fus.L * Fus.isEnable ...
            + HorStab.L * HorStab.isEnable ...
            + VerStab.L * VerStab.isEnable;
Fnet(5) = LowerRotor.M ...
            + UpperRotor.M ...
            + Prop.M * Prop.isEnable ...
            + Fus.M * Fus.isEnable ...
            + HorStab.M * HorStab.isEnable ...
            + VerStab.M * VerStab.isEnable;
Fnet(6) = LowerRotor.N ...
            + UpperRotor.N ... 
            + Prop.N * Prop.isEnable ...
            + Fus.N * Fus.isEnable ...
            + HorStab.N * HorStab.isEnable ...
            + VerStab.N * VerStab.isEnable;
        
% Fnet(7) = LowerRotor.T_blade_element - LowerRotor.T_momentum_theory;
% Fnet(8) = UpperRotor.T_blade_element - UpperRotor.T_momentum_theory;

% 返回结构体
Rotorcraft.DoubleRotorHelicopter    = DoubleRotorHelicopter;
Rotorcraft.LowerRotor               = LowerRotor;
Rotorcraft.UpperRotor               = UpperRotor;
Rotorcraft.Prop                     = Prop;
Rotorcraft.Fus                      = Fus;
Rotorcraft.HorStab                  = HorStab;  
Rotorcraft.VerStab                  = VerStab;

end

