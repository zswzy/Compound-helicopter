function Fnet = Aerodynamics_trim_2rotor_prop_8var(x)
%AERODYNAMICS_TRIM_2ROTOR_8VAR 
%   用于悬停/前飞配平 
%   X,Y,Z,L,M,N=0,
%   [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
global DoubleRotorHelicopter
global LowerRotor UpperRotor
global Prop 

theta_0     = x(1);
theta_diff  = x(2);
theta_1c    = x(3);
theta_1s    = x(4);
theta       = x(5);
phi         = x(6);
v_i1        = x(7); % 下旋翼固有诱导速度
v_i2        = x(8); % 上旋翼固有诱导速度

% 变量赋值/初始化
LowerRotor.v_i      = v_i1;
LowerRotor.theta_0  = theta_0 - theta_diff;
LowerRotor.theta_1c = theta_1c;
LowerRotor.theta_1s = theta_1s;
UpperRotor.v_i      = v_i2;
UpperRotor.theta_0  = theta_0 + theta_diff;
UpperRotor.theta_1c = -theta_1c;
UpperRotor.theta_1s = theta_1s;

LowerRotor.v_0      = LowerRotor.v_i + LowerRotor.delta_1 * UpperRotor.v_i;
UpperRotor.v_0      = UpperRotor.v_i + UpperRotor.delta_2 * LowerRotor.v_i;

% 计算挥舞，力，力矩
LowerRotor.calculate_flapping_angle_3blades();
LowerRotor.calculate_force();
LowerRotor.calculate_torque();
UpperRotor.calculate_flapping_angle_3blades();
UpperRotor.calculate_force();
UpperRotor.calculate_torque();
Prop.calculate_force();
Prop.calculate_torque();

% 构建方程
Fnet(1) = LowerRotor.X ...
            + UpperRotor.X ...
            + Prop.X * Prop.isEnable ...
            - DoubleRotorHelicopter.GWF*sin(theta);
Fnet(2) = LowerRotor.Y ...
            + UpperRotor.Y ...
            + Prop.Y * Prop.isEnable ...
            + DoubleRotorHelicopter.GWF*cos(theta)*sin(phi);
Fnet(3) = LowerRotor.Z ...
            + UpperRotor.Z ...
            + Prop.Z * Prop.isEnable ...
            + DoubleRotorHelicopter.GWF*cos(theta)*cos(phi);
Fnet(4) = LowerRotor.L ...
            + UpperRotor.L ...
            + Prop.L * Prop.isEnable;
Fnet(5) = LowerRotor.M ...
            + UpperRotor.M ...
            + Prop.M * Prop.isEnable;
Fnet(6) = LowerRotor.N ...
            + UpperRotor.N ...
            + Prop.N * Prop.isEnable;
        
Fnet(7) = LowerRotor.Z_h_blade_element - LowerRotor.Z_h_momentum_theory;
Fnet(8) = UpperRotor.Z_h_blade_element - UpperRotor.Z_h_momentum_theory;


end

