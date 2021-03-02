function [power_total,Fnet] = trim_power_prop_full_var(Rotorcraft,x)
%TRIM_POWER_PROP power_total = trim_power_prop(Prop_theta_0,Prop_isEnable)
%   输出整机功率
%   Prop_theta_0 (rad)
%   Prop_isEnable = 0(default) or 1

%   x = Prop_theta_0,Prop_isEnable,delta_e,delta_r,theta_1c_diff,theta_1s_diff,theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2

Prop_theta_0    = x(1);
Prop_isEnable   = x(2);
delta_e         = x(3);
delta_r         = x(4);
theta_1c_diff   = x(5);
theta_1s_diff   = x(6);
theta_0         = x(7);
theta_diff      = x(8);
theta_1c        = x(9);
theta_1s        = x(10);
theta           = x(11);
phi             = x(12);
v_i1            = x(13);
v_i2            = x(14);

Rotorcraft.Prop.theta_0 = Prop_theta_0;
Rotorcraft.Prop.isEnable = Prop_isEnable;
Rotorcraft.HorStab.delta_e = delta_e;
Rotorcraft.VerStab.delta_r = delta_r;
Rotorcraft.LowerRotor.theta_1c_diff = theta_1c_diff;
Rotorcraft.LowerRotor.theta_1s_diff = theta_1s_diff;
Rotorcraft.UpperRotor.theta_1c_diff = theta_1c_diff;
Rotorcraft.UpperRotor.theta_1s_diff = theta_1s_diff;

x_trim = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2];

[Rotorcraft,Fnet] = Aerodynamics_full_8var(Rotorcraft, x_trim);

power_total = Rotorcraft.LowerRotor.Power_total ...
            + Rotorcraft.UpperRotor.Power_total ...
            + Rotorcraft.Prop.Power_resist ...
            + Rotorcraft.Prop.Power_induced * Prop_isEnable;

end

