function power_total = trim_power_prop(Rotorcraft,Prop_theta_0,Prop_isEnable,delta_e,delta_r,theta_1c_diff,theta_1s_diff)
%TRIM_POWER_PROP power_total = trim_power_prop(Rotorcraft,Prop_theta_0,Prop_isEnable,delta_e,delta_r,theta_1c_diff,theta_1s_diff)
%   输出整机功率
%   Prop_theta_0 (rad)
%   Prop_isEnable = 0(default) or 1

if nargin == 3
    delta_e         = 0;
    delta_r         = 0;
    theta_1c_diff   = 0;
    theta_1s_diff   = 0;
else
end

% x = [theta_0,theta_diff,theta_1c,theta_1s,theta,phi,v_i1,v_i2]
options         = optimset('Display','off','TolFun',1e-15,'Maxiter',100,'Algorithm','levenberg-marquardt' ,'MaxFunEvals',20000);
cell_InitialStates      = {[0.01,0,0,0,0,0,10,10],[0.01,0,0,0,0,0,3,3], ...
                            [0.1,0,0,0,0,0,10,10],[0.1,0,0,0,0,0,3,3], ...
                            [0.2,0,0,0,0,0,10,10],[0.2,0,0,0,0,0,3,3], ...
                            [0.3,0,0,0,0,0,10,10],[0.3,0,0,0,0,0,3,3]};
[x_trim,~,exitflag,~,Rotorcraft,~,~] = trim_solve(Rotorcraft, ...
                                @Aerodynamics_trim_full_8var, ... 
                                cell_InitialStates, ...
                                options, ...
                                2, ...                  % LowerRotor.inteference
                                2, ...                  % UpperRotor.inteference
                                Prop_theta_0, ...         % Prop.theta_0
                                Prop_isEnable, ...                  % Prop.isEnable
                                1, ...                  % Fus.isEnable
                                delta_e, ...         % HorStab.delta_e
                                1, ...                  % HorStab.isEnable
                                delta_r, ...          % VerStab.delta_r
                                1, ...                  % VerStab.isEnable
                                theta_1c_diff, ...         % theta_1c_diff
                                theta_1s_diff);            % theta_1s_diff

% power_total = Rotorcraft.LowerRotor.Power_total ...
%                 + Rotorcraft.UpperRotor.Power_total ...
%                 + Rotorcraft.Fus.Power_total ...
%                 + Rotorcraft.Prop.Power_resist ...
%                 + Rotorcraft.Prop.Power_induced * Prop_isEnable;
if exitflag > 0
    power_total = Rotorcraft.LowerRotor.Power_total ...
                + Rotorcraft.UpperRotor.Power_total ...
                + Rotorcraft.Prop.Power_resist ...
                + Rotorcraft.Prop.Power_induced * Prop_isEnable;
else
    power_total = inf;
end
end

