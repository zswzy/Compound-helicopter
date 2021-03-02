function Fnet = Aerodynamics_trim_2rotor_3var(x)
%AERODYNAMICS_TRIM_2ROTOR_8VAR(x)
%   用于悬停配平 Z=0,[theta_0,v_i1,v_i2]
global inteference
global DoubleRotorHelicopter
global LowerRotor UpperRotor

theta_0     = x(1);
v_i1        = x(2);
v_i2        = x(3);

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
LowerRotor.theta_0  = theta_0;
LowerRotor.calculate_flapping_angle_3blades();
LowerRotor.calculate_force();

UpperRotor.v_i      = v_i2;
UpperRotor.theta_0  = theta_0;
UpperRotor.calculate_flapping_angle_3blades();
UpperRotor.calculate_force();

Fnet(1) = LowerRotor.Z + UpperRotor.Z + DoubleRotorHelicopter.GWF;
Fnet(2) = LowerRotor.Z_h_blade_element - LowerRotor.Z_h_momentum_theory;
Fnet(3) = UpperRotor.Z_h_blade_element - UpperRotor.Z_h_momentum_theory;

end

