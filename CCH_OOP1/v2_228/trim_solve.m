function [x_trim,fval,exitflag,output,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft, ...
                                                trim_function, ...
                                                cell_InitialStates, ...
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
%TRIM_SOLVE [x,fval,exitflag,output,Rotorcraft,Fnet,power_total] = trim_solve(Rotorcraft,trim_function,InitialStates,options,LowerRotor_inteference,UpperRotor_inteference,Prop_theta_0,Prop_isEnable)
%   [x_trim,fval,exitflag,output]:   fsolve output
%   Rotorcraft:                 Rotorcraft 结构体
%   Fnet:                       [X,Y,Z,L,M,N]
%   power_total:                total power
%
%   Rotorcraft:                 struct
%   trim_function:              fsolve used function @+func_name eg:@Aerodynamics_trim_2rotor_prop_8var
%   cell_InitialStates:         a cell of fsolve initial states
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

LowerRotor              = Rotorcraft.LowerRotor;
UpperRotor              = Rotorcraft.UpperRotor;
Prop                    = Rotorcraft.Prop;
Fus                     = Rotorcraft.Fus;
HorStab                 = Rotorcraft.HorStab;
VerStab                 = Rotorcraft.VerStab;

switch nargin 
    case 4
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
    case 5
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
    case 6
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
    case 8
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
    case 9
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
    case 11
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
    case 13
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
    case 15
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

% call fsolve, if not solved, try different initial point.
cell_InitialStates_size = size(cell_InitialStates);
jmax = cell_InitialStates_size(2);
j = 1;
exitflag = -1;
theta_0 = -1;
while (exitflag <= 0 || theta_0 < 0) && j <= jmax
    problem = struct;
    problem.options = options;
    problem.solver = 'fsolve';
    problem.objective = @(x) trim_function(Rotorcraft, x);
    problem.x0 = cell_InitialStates{j};
    [x_trim,fval,exitflag,output] = fsolve(problem);
    theta_0 = x_trim(1);
    j = j+1;
end
[Rotorcraft,Fnet] = Aerodynamics_full_8var(Rotorcraft, x_trim);

power_total = calculate_power(Rotorcraft);
end

