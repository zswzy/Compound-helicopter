function [x,fval,exitflag,output,power_total] = trim_solve(trim_function, ...
                                                InitialStates, ...
                                                options, ...
                                                LowerRotor_inteference, ...
                                                UpperRotor_inteference, ...
                                                Prop_theta_0, ...
                                                Prop_isEnable, ...
                                                Fus_isEnable, ...
                                                HorStab_delta_e, ...
                                                HorStab_isEnable, ...
                                                VerStab_delta_r, ...
                                                VerStab_isEnable, ...
                                                theta_1c_diff, ...
                                                theta_1s_diff)
%TRIM_SOLVE [x,fval,exitflag,output] = trim_solve(trim_function,InitialStates,options,LowerRotor_inteference,UpperRotor_inteference,Prop_theta_0,Prop_isEnable)
%   [x,fval,exitflag,output]:   fsolve output
%   power_total:                total power
%   trim_function:              fsolve used function @+func_name eg:@Aerodynamics_trim_2rotor_prop_8var
%   InitialStates:              fsolve initial states
%   options:                    fsolve options
%   LowerRotor_inteference:     LowerRotor.inteference = 0(default), 1 or 2
%   UpperRotor_inteference:     UpperRotor.inteference = 0(default), 1 or 2
%   Prop_theta_0:               Prop.theta_0 (rad)
%   Prop_isEnable:              Prop.isEnable = 0(default) or 1
%   Fus_isEnable:               Fus.isEnable = 0(default) or 1
%   HorStab_delta_e:            HorStab.delta_e = HorStab_delta_e (rad)
%   HorStab_isEnable:           HorStab.isEnable = 0(default) or 1
%   VerStab_delta_r:            VerStab.delta_r = VerStab_delta_r (rad)
%   VerStab_isEnable:           VerStab.isEnable = 0(default) or 1
%   theta_1c_diff:              
%   theta_1s_diff:              

global LowerRotor UpperRotor
global Prop 
global Fus
global HorStab 
global VerStab

switch nargin 
    case 3
        LowerRotor.inteference      = 0;    % set default
        UpperRotor.inteference      = 0;    % set default
        Prop.isEnable               = 0;    % set default
        Prop.theta_0                = 0;    % set default
        Fus.isEnable                = 0;    % set default
        HorStab.delta_e             = 0;    % set default
        HorStab.isEnable            = 0;    % set default
        VerStab.delta_r             = 0;    % set default
        VerStab.isEnable            = 0;    % set default
        LowerRotor.theta_1c_diff    = 0;    % set default
        LowerRotor.theta_1s_diff    = 0;    % set default
        UpperRotor.theta_1c_diff    = 0;    % set default
        UpperRotor.theta_1s_diff    = 0;    % set default
    case 4
        LowerRotor.inteference      = LowerRotor_inteference;
        UpperRotor.inteference      = 0;    % set default
        Prop.isEnable               = 0;    % set default
        Prop.theta_0                = 0;    % set default
        Fus.isEnable                = 0;    % set default
        HorStab.delta_e             = 0;    % set default
        HorStab.isEnable            = 0;    % set default
        VerStab.delta_r             = 0;    % set default
        VerStab.isEnable            = 0;    % set default
        LowerRotor.theta_1c_diff    = 0;    % set default
        LowerRotor.theta_1s_diff    = 0;    % set default
        UpperRotor.theta_1c_diff    = 0;    % set default
        UpperRotor.theta_1s_diff    = 0;    % set default
    case 5
        LowerRotor.inteference      = LowerRotor_inteference;
        UpperRotor.inteference      = UpperRotor_inteference;
        Prop.isEnable               = 0;    % set default
        Prop.theta_0                = 0;    % set default
        Fus.isEnable                = 0;    % set default
        HorStab.delta_e             = 0;    % set default
        HorStab.isEnable            = 0;    % set default
        VerStab.delta_r             = 0;    % set default
        VerStab.isEnable            = 0;    % set default
        LowerRotor.theta_1c_diff    = 0;    % set default
        LowerRotor.theta_1s_diff    = 0;    % set default
        UpperRotor.theta_1c_diff    = 0;    % set default
        UpperRotor.theta_1s_diff    = 0;    % set default
    case 7
        LowerRotor.inteference      = LowerRotor_inteference;
        UpperRotor.inteference      = UpperRotor_inteference;
        Prop.isEnable               = Prop_isEnable;
        Prop.theta_0                = Prop_theta_0;
        Fus.isEnable                = 0;    % set default
        HorStab.delta_e             = 0;    % set default
        HorStab.isEnable            = 0;    % set default
        VerStab.delta_r             = 0;    % set default
        VerStab.isEnable            = 0;    % set default
        LowerRotor.theta_1c_diff    = 0;    % set default
        LowerRotor.theta_1s_diff    = 0;    % set default
        UpperRotor.theta_1c_diff    = 0;    % set default
        UpperRotor.theta_1s_diff    = 0;    % set default
    case 8
        LowerRotor.inteference      = LowerRotor_inteference;
        UpperRotor.inteference      = UpperRotor_inteference;
        Prop.isEnable               = Prop_isEnable;
        Prop.theta_0                = Prop_theta_0;
        Fus.isEnable                = Fus_isEnable;
        HorStab.delta_e             = 0;    % set default
        HorStab.isEnable            = 0;    % set default
        VerStab.delta_r             = 0;    % set default
        VerStab.isEnable            = 0;    % set default
        LowerRotor.theta_1c_diff    = 0;    % set default
        LowerRotor.theta_1s_diff    = 0;    % set default
        UpperRotor.theta_1c_diff    = 0;    % set default
        UpperRotor.theta_1s_diff    = 0;    % set default
    case 10
        LowerRotor.inteference      = LowerRotor_inteference;
        UpperRotor.inteference      = UpperRotor_inteference;
        Prop.isEnable               = Prop_isEnable;
        Prop.theta_0                = Prop_theta_0;
        Fus.isEnable                = Fus_isEnable;
        HorStab.delta_e             = HorStab_delta_e;
        HorStab.isEnable            = HorStab_isEnable;
        VerStab.delta_r             = 0;    % set default
        VerStab.isEnable            = 0;    % set default
        LowerRotor.theta_1c_diff    = 0;    % set default
        LowerRotor.theta_1s_diff    = 0;    % set default
        UpperRotor.theta_1c_diff    = 0;    % set default
        UpperRotor.theta_1s_diff    = 0;    % set default
    case 12
        LowerRotor.inteference      = LowerRotor_inteference;
        UpperRotor.inteference      = UpperRotor_inteference;
        Prop.isEnable               = Prop_isEnable;
        Prop.theta_0                = Prop_theta_0;
        Fus.isEnable                = Fus_isEnable;
        HorStab.delta_e             = HorStab_delta_e;
        HorStab.isEnable            = HorStab_isEnable;
        VerStab.delta_r             = VerStab_delta_r;
        VerStab.isEnable            = VerStab_isEnable;
        LowerRotor.theta_1c_diff    = 0;    % set default
        LowerRotor.theta_1s_diff    = 0;    % set default
        UpperRotor.theta_1c_diff    = 0;    % set default
        UpperRotor.theta_1s_diff    = 0;    % set default
    case 14
        LowerRotor.inteference      = LowerRotor_inteference;
        UpperRotor.inteference      = UpperRotor_inteference;
        Prop.isEnable               = Prop_isEnable;
        Prop.theta_0                = Prop_theta_0;
        Fus.isEnable                = Fus_isEnable;
        HorStab.delta_e             = HorStab_delta_e;
        HorStab.isEnable            = HorStab_isEnable;
        VerStab.delta_r             = VerStab_delta_r;
        VerStab.isEnable            = VerStab_isEnable;
        LowerRotor.theta_1c_diff    = theta_1c_diff;
        LowerRotor.theta_1s_diff    = theta_1s_diff;
        UpperRotor.theta_1c_diff    = theta_1c_diff;
        UpperRotor.theta_1s_diff    = theta_1s_diff;
end

% call fsolve
[x,fval,exitflag,output] = fsolve(trim_function,InitialStates,options);
power_total = LowerRotor.Power_total ...
                + UpperRotor.Power_total ...
                + Fus.Power_total ...
                + Prop.Power_resist ...
                + Prop.Power_induced * Prop_isEnable;
end

