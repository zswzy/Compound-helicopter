function Fnet = Aerodynamics_trim_2rotor_8var(x)
%AERODYNAMICS_TRIM_2ROTOR_8VAR 
%   用于悬停/前飞配平 
%   X,Y,Z,L,M,N=0,
%   [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
global inteference
global DoubleRotorHelicopter
global LowerRotor UpperRotor

theta_0     = x(1);
theta_diff  = x(2);
theta_1c    = x(3);
theta_1s    = x(4);
theta       = x(5);
phi         = x(6);
v_i1        = x(7); % 下旋翼固有诱导速度
v_i2        = x(8); % 上旋翼固有诱导速度

if inteference == 0
    LowerRotor.v_0      = v_i1; 
    UpperRotor.v_0      = v_i2;
elseif inteference == 1
    LowerRotor.v_0      = v_i1 + v_i2; 
    UpperRotor.v_0      = v_i2;
elseif inteference == 2
    LowerRotor.v_0      = v_i1 + LowerRotor.delta_1*v_i2; 
    UpperRotor.v_0      = v_i2 + UpperRotor.delta_2*v_i2;
end

LowerRotor.v_i      = v_i1;
LowerRotor.theta_0  = theta_0 - theta_diff;
LowerRotor.theta_1c = theta_1c;
LowerRotor.theta_1s = theta_1s;
LowerRotor.calculate_flapping_angle_3blades();
LowerRotor.calculate_force();
LowerRotor.calculate_torque();

UpperRotor.v_i      = v_i2;
UpperRotor.theta_0  = theta_0 + theta_diff;
UpperRotor.theta_1c = -theta_1c;
UpperRotor.theta_1s = theta_1s;
UpperRotor.calculate_flapping_angle_3blades();
UpperRotor.calculate_force();
UpperRotor.calculate_torque();


Fnet(1) = LowerRotor.X + UpperRotor.X - DoubleRotorHelicopter.GWF*sin(theta);
Fnet(2) = LowerRotor.Y + UpperRotor.Y + DoubleRotorHelicopter.GWF*cos(theta)*sin(phi);
Fnet(3) = LowerRotor.Z + UpperRotor.Z + DoubleRotorHelicopter.GWF*cos(theta)*cos(phi);
Fnet(4) = LowerRotor.L + UpperRotor.L;
Fnet(5) = LowerRotor.M + UpperRotor.M;
Fnet(6) = LowerRotor.N + UpperRotor.N;
Fnet(7) = LowerRotor.Z_h_blade_element - LowerRotor.Z_h_momentum_theory;
Fnet(8) = UpperRotor.Z_h_blade_element - UpperRotor.Z_h_momentum_theory;


end

