function power_total = trim_power_prop(Prop_theta_0,Prop_isEnable)
%TRIM_POWER_PROP power_total = trim_power_prop(Prop_theta_0,Prop_isEnable)
%   输出整机功率
%   Prop_theta_0 (rad)
%   Prop_isEnable = 0(default) or 1

global LowerRotor UpperRotor 
global Prop
global Fus

if nargin == 1
    Prop_isEnable = 1;
else
end
% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options         = optimset('Display','off','TolFun',1e-15,'Maxiter',5000,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
InitialStates   = [0.01,0,0,0,0,0,10,10];
[x,~,~,~] = trim_solve(@Aerodynamics_trim_full_8var, ... 
                                InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                Prop_theta_0, ...         % Prop.theta_0
                                Prop_isEnable, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                deg2rad(0), ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                deg2rad(0), ...          % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                deg2rad(0), ...         % theta_1c_diff
                                deg2rad(0));            % theta_1s_diff% theta_1s_diff
power_total = LowerRotor.Power_total ...
                + UpperRotor.Power_total ...
                + Fus.Power_total ...
                + Prop.Power_resist ...
                + Prop.Power_induced * Prop_isEnable;
%display(power_total)
end

