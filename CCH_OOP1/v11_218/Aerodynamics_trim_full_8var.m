function Fnet = Aerodynamics_trim_full_8var(x)
%AERODYNAMICS_TRIM_FULL_8VAR 
%   用于悬停/前飞配平 
%   X,Y,Z,L,M,N=0,
%   [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
global DoubleRotorHelicopter
global LowerRotor UpperRotor 
global Prop
global Fus
global HorStab
global VerStab

theta_0     = x(1);
theta_diff  = x(2);
theta_1c    = x(3);
theta_1s    = x(4);
theta       = x(5);
phi         = x(6);
v_i1        = x(7); % 下旋翼固有诱导速度
v_i2        = x(8); % 上旋翼固有诱导速度

% 机体姿态角
DoubleRotorHelicopter.theta = theta;
DoubleRotorHelicopter.phi   = phi;
DoubleRotorHelicopter.psi   = 0;

% 计算当地速度
DoubleRotorHelicopter.calculate_local_variables();
copy_states();

% 变量赋值/初始化
LowerRotor.v_i      = v_i1;
LowerRotor.theta_0  = theta_0 - theta_diff;
LowerRotor.theta_1c = theta_1c - LowerRotor.theta_1c_diff;
LowerRotor.theta_1s = theta_1s - LowerRotor.theta_1s_diff;
UpperRotor.v_i      = v_i2;
UpperRotor.theta_0  = theta_0 + theta_diff;
UpperRotor.theta_1c = -(theta_1c + LowerRotor.theta_1c_diff);
UpperRotor.theta_1s = theta_1s + LowerRotor.theta_1s_diff;
% 旋翼干扰诱导速度
LowerRotor.v_0      = LowerRotor.v_i + LowerRotor.delta_1 * UpperRotor.v_i;
UpperRotor.v_0      = UpperRotor.v_i + UpperRotor.delta_2 * LowerRotor.v_i;

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

% 构建方程 X,Y,Z,L,M,N = 0
Fnet(1) = LowerRotor.X ...
            + UpperRotor.X ...
            + Prop.X * Prop.isEnable ...
            + Fus.X * Fus.isEnable ...
            + HorStab.X * HorStab.isEnable ...
            + VerStab.X * VerStab.isEnable ...
            - DoubleRotorHelicopter.GWF*sin(theta);
Fnet(2) = LowerRotor.Y ...
            + UpperRotor.Y ...
            + Prop.Y * Prop.isEnable ...
            + Fus.Y * Fus.isEnable ...
            + HorStab.Y * HorStab.isEnable ...
            + VerStab.Y * VerStab.isEnable ...
            + DoubleRotorHelicopter.GWF*cos(theta)*sin(phi);
Fnet(3) = LowerRotor.Z ...
            + UpperRotor.Z ...
            + Prop.Z * Prop.isEnable ...
            + Fus.Z * Fus.isEnable ...
            + HorStab.Z * HorStab.isEnable ...
            + VerStab.Z * VerStab.isEnable ...
            + DoubleRotorHelicopter.GWF*cos(theta)*cos(phi);
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
        
Fnet(7) = LowerRotor.T_blade_element - LowerRotor.T_momentum_theory;
Fnet(8) = UpperRotor.T_blade_element - UpperRotor.T_momentum_theory;


end

